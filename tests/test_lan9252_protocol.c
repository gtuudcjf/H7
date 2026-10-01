#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "lan9252_spi.h"

/* These fixtures are literal SPI bytes, derived from DS00001909A chapters
 * 10 and 12. No production register constants/builders are used by the fake. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
typedef struct { uint8_t tx[80], rx[80]; size_t length; } Frame;
typedef struct {
    Frame frames[256];
    unsigned count, current, position, selected, selects, deselects, calls;
    unsigned fail_call, late_call, repeat_last, max_calls;
    LAN9252Status failure;
    uint32_t now, step;
} Fake;
static Fake f;
static LAN9252Context ctx;
static unsigned cases;

static size_t hex(uint8_t *out, const char *s)
{
    size_t n = 0;
    while (*s) {
        unsigned v;
        while (*s == ' ') ++s;
        if (!*s) break;
        CHECK(sscanf(s, "%2x", &v) == 1);
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}
static void frame(const char *tx, const char *rx)
{
    Frame *p;
    CHECK(f.count < 256);
    p = &f.frames[f.count++];
    p->length = hex(p->tx, tx);
    if (rx) CHECK(hex(p->rx, rx) == p->length);
}
static void select_spi(void *p)
{
    Fake *v = p;
    CHECK(!v->selected);
    CHECK(v->current < v->count);
    v->selected = 1; v->position = 0; ++v->selects;
}
static void deselect_spi(void *p)
{
    Fake *v = p;
    if (!v->selected) return; /* InitContext also drives the idle level. */
    if (!v->fail_call && !v->late_call)
        CHECK(v->position == v->frames[v->current].length);
    v->selected = 0; ++v->deselects;
    if (!(v->repeat_last && v->current + 1 == v->count)) ++v->current;
}
static uint32_t time_us(void *p)
{
    Fake *v = p;
    uint32_t result = v->now;
    v->now += v->step;
    return result;
}
static LAN9252Status transfer(void *p, const uint8_t *tx, uint8_t *rx,
                              size_t n, uint32_t timeout)
{
    Fake *v = p;
    Frame *s = &v->frames[v->current];
    size_t i;
    CHECK(v->selected && n && timeout && timeout <= 100000);
    CHECK(++v->calls < v->max_calls); /* Missing deadlines fail, never hang. */
    if (v->calls == v->fail_call) return v->failure;
    CHECK(v->position + n <= s->length);
    for (i = 0; i < n; ++i) {
        CHECK((tx ? tx[i] : 0) == s->tx[v->position]);
        if (rx) rx[i] = s->rx[v->position];
        ++v->position;
    }
    if (v->calls == v->late_call) v->now += timeout;
    return LAN9252_OK;
}
static void reset(void)
{
    LAN9252Transport t;
    memset(&f, 0, sizeof(f));
    f.step = 1; f.max_calls = 100000; f.failure = LAN9252_ERR_IO;
    t.transfer = transfer; t.select = select_spi; t.deselect = deselect_spi;
    t.time_us = time_us; t.context = &f;
    LAN9252_InitContext(&ctx, &t);
    CHECK(ctx.last_error == LAN9252_OK && ctx.timeout_count == 0);
}
static void result(LAN9252Status got, LAN9252Status want, int timed_out)
{
    CHECK(got == want && ctx.last_error == want);
    CHECK(ctx.timeout_count == (uint32_t)timed_out);
    CHECK(!f.selected && f.selects == f.deselects);
    if (want == LAN9252_OK) CHECK(f.current == f.count);
    ++cases;
}
static void csr_idle(void) { frame("0b 03 04 00 00 00 00 00", NULL); }
static void csr_read(const char *command, const char *data)
{
    csr_idle(); frame(command, NULL); csr_idle();
    frame("0b 03 00 00 00 00 00 00", data);
}
static void csr_write(const char *data, const char *command)
{
    csr_idle(); frame(data, NULL); frame(command, NULL); csr_idle();
}
static void test_direct(void)
{
    uint8_t b[6] = {0xee,0,0,0,0,0xdd};
    reset();
    frame("0b 12 34 00 00 00 00 00", "00 00 00 00 21 43 65 87");
    result(LAN9252_ReadDirect(&ctx, 0x1234, b + 1, 4), LAN9252_OK, 0);
    CHECK(!memcmp(b, "\xee\x21\x43\x65\x87\xdd", 6));
    reset(); frame("02 12 34 21 43 65 87", NULL);
    result(LAN9252_WriteDirect(&ctx, 0x1234, b + 1, 4), LAN9252_OK, 0);
    CHECK(!memcmp(b, "\xee\x21\x43\x65\x87\xdd", 6));
    reset(); frame("0b 3f fc 00 00 00 00 00", "00 00 00 00 21 43 65 87");
    result(LAN9252_ReadDirect(&ctx, 0x3ffc, b + 1, 4), LAN9252_OK, 0);
    CHECK(!memcmp(b, "\xee\x21\x43\x65\x87\xdd", 6));
    reset(); frame("02 3f fc 21 43 65 87", NULL);
    result(LAN9252_WriteDirect(&ctx, 0x3ffc, b + 1, 4), LAN9252_OK, 0);
}
static void test_direct_rejects_invalid_transactions(void)
{
    static const size_t lengths[] = {1,0,2,3,5,6,8,(size_t)-1};
    static const uint16_t addresses[] = {
        0x1235,0x1236,0x1237,0x3ffd,0x3ffe,0x3fff,0x4000,0x8000,0xc000,0xfffc
    };
    uint8_t b[10]; unsigned i; int write;
    /* Removing validation would issue SPI traffic for incomplete DWORDs,
     * unaligned registers, or an encoded SPI address mode. */
    for(write=0;write<2;++write) for(i=0;i<sizeof(lengths)/sizeof(lengths[0]);++i) {
        LAN9252Status status;
        reset();memset(b,0xa5,sizeof(b));
        /* A valid wire fixture for the first regression makes old code return
         * OK, proving the failure is API acceptance rather than fake setup. */
        if(i==0) frame(write ? "02 12 34 a5" : "0b 12 34 00 00",
                       write ? NULL : "00 00 00 00 a5");
        status = write ? LAN9252_WriteDirect(&ctx,0x1234,b+1,lengths[i]) :
                         LAN9252_ReadDirect(&ctx,0x1234,b+1,lengths[i]);
        if(status!=LAN9252_ERR_ARGUMENT)
            fprintf(stderr,"invalid Direct length %lu accepted: status=%d\n",(unsigned long)lengths[i],(int)status);
        result(status,LAN9252_ERR_ARGUMENT,0);
        CHECK(f.calls==0 && f.selects==0);
        CHECK(!memcmp(b,"\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5",10));
    }
    for(write=0;write<2;++write) for(i=0;i<sizeof(addresses)/sizeof(addresses[0]);++i) {
        reset();memset(b,0xa5,sizeof(b));
        result(write ? LAN9252_WriteDirect(&ctx,addresses[i],b+1,4) :
                       LAN9252_ReadDirect(&ctx,addresses[i],b+1,4),LAN9252_ERR_ARGUMENT,0);
        CHECK(f.calls==0 && f.selects==0);
        CHECK(!memcmp(b,"\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5\xa5",10));
    }
}
static void test_csr(void)
{
    uint8_t b[10] = {0xee, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0xdd};
    reset();
    memset(b + 1, 0xaa, 6);
    csr_read("02 03 04 21 01 01 c0", "00 00 00 00 11 99 99 99");
    csr_read("02 03 04 22 01 02 c0", "00 00 00 00 22 33 99 99");
    csr_read("02 03 04 24 01 02 c0", "00 00 00 00 44 55 99 99");
    csr_read("02 03 04 26 01 01 c0", "00 00 00 00 66 99 99 99");
    result(LAN9252_ReadEsc(&ctx, 0x0121, b + 1, 6), LAN9252_OK, 0);
    CHECK(!memcmp(b, "\xee\x11\x22\x33\x44\x55\x66\xdd", 8));
    reset();
    csr_write("02 03 00 11 00 00 00", "02 03 04 21 01 01 80");
    csr_write("02 03 00 22 33 00 00", "02 03 04 22 01 02 80");
    csr_write("02 03 00 44 55 00 00", "02 03 04 24 01 02 80");
    csr_write("02 03 00 66 00 00 00", "02 03 04 26 01 01 80");
    result(LAN9252_WriteEsc(&ctx, 0x0121, b + 1, 6), LAN9252_OK, 0);
    reset();
    csr_read("02 03 04 00 02 04 c0", "00 00 00 00 12 34 56 78");
    result(LAN9252_ReadEsc(&ctx, 0x0200, b + 1, 4), LAN9252_OK, 0);
    CHECK(!memcmp(b + 1, "\x12\x34\x56\x78", 4));
    reset();
    csr_write("02 03 00 12 34 56 78", "02 03 04 00 02 04 80");
    result(LAN9252_WriteEsc(&ctx, 0x0200, b + 1, 4), LAN9252_OK, 0);
}
typedef struct {
    uint16_t address; size_t length;
    const char *read_len, *write_len, *r1, *r2, *w1, *w2;
} PramCase;
static const PramCase pram_cases[] = {
    {0x1000,1,"02 03 08 00 10 01 00","02 03 10 00 10 01 00", "00 00 00 00 11 aa bb cc",NULL,"02 00 20 11 00 00 00",NULL},
    {0x1001,2,"02 03 08 01 10 02 00","02 03 10 01 10 02 00", "00 00 00 00 aa 11 22 cc",NULL,"02 00 20 00 11 22 00",NULL},
    {0x1002,3,"02 03 08 02 10 03 00","02 03 10 02 10 03 00", "00 00 00 00 aa bb 11 22","00 00 00 00 33 bb cc dd","02 00 20 00 00 11 22","02 00 20 33 00 00 00"},
    {0x1000,4,"02 03 08 00 10 04 00","02 03 10 00 10 04 00", "00 00 00 00 11 22 33 44",NULL,"02 00 20 11 22 33 44",NULL},
    {0x1400,6,"02 03 08 00 14 06 00","02 03 10 00 14 06 00", "00 00 00 00 11 22 33 44","00 00 00 00 55 66 cc dd","02 00 20 11 22 33 44","02 00 20 55 66 00 00"},
    {0x1003,4,"02 03 08 03 10 04 00","02 03 10 03 10 04 00", "00 00 00 00 aa bb cc 11","00 00 00 00 22 33 44 dd","02 00 20 00 00 00 11","02 00 20 22 33 44 00"},
    {0x1100,2,"02 03 08 00 11 02 00","02 03 10 00 11 02 00", "00 00 00 00 11 22 cc dd",NULL,"02 00 20 11 22 00 00",NULL},
    {0x1fff,1,"02 03 08 ff 1f 01 00","02 03 10 ff 1f 01 00", "00 00 00 00 aa bb cc 11",NULL,"02 00 20 00 00 00 11",NULL}
};
static void pram_start(int write, const PramCase *p)
{
    frame(write ? "02 03 14 00 00 00 40" : "02 03 0c 00 00 00 40", NULL);
    frame(write ? "0b 03 14 00 00 00 00 00" : "0b 03 0c 00 00 00 00 00", NULL);
    frame(write ? p->write_len : p->read_len, NULL);
    frame(write ? "02 03 14 00 00 00 80" : "02 03 0c 00 00 00 80", NULL);
}
static void pram_script(int write, const PramCase *p)
{
    const char *poll = write ? "0b 03 14 00 00 00 00 00" : "0b 03 0c 00 00 00 00 00";
    pram_start(write, p);
    /* Available bit alone is insufficient: count=0 must never touch FIFO.
     * Count alone is insufficient too: a deasserted available bit must wait. */
    frame(poll, "00 00 00 00 01 00 00 80");
    frame(poll, "00 00 00 00 00 01 00 80");
    frame(poll, "00 00 00 00 01 01 00 80");
    frame(write ? p->w1 : "0b 00 04 00 00 00 00 00", write ? NULL : p->r1);
    if (p->r2) {
        frame(poll, "00 00 00 00 00 00 00 80");
        frame(poll, "00 00 00 00 01 01 00 80");
        frame(write ? p->w2 : "0b 00 04 00 00 00 00 00", write ? NULL : p->r2);
    }
    frame(poll, "00 00 00 00 00 00 00 80");
    frame(poll, NULL);
}
static void test_pram(void)
{
    unsigned i; int write;
    for (i = 0; i < sizeof(pram_cases)/sizeof(pram_cases[0]); ++i) {
        const PramCase *p = &pram_cases[i];
        for (write = 0; write < 2; ++write) {
            uint8_t b[8] = {0xee,0x11,0x22,0x33,0x44,0x55,0x66,0xdd};
            reset(); pram_script(write,p);
            if (!write) memset(b + 1, 0xaa, p->length);
            result(write ? LAN9252_WriteEsc(&ctx,p->address,b+1,p->length) :
                           LAN9252_ReadEsc(&ctx,p->address,b+1,p->length), LAN9252_OK,0);
            CHECK(!memcmp(b+1,"\x11\x22\x33\x44\x55\x66",p->length));
            CHECK(!memcmp(b,"\xee\x11\x22\x33\x44\x55\x66\xdd",8));
        }
    }
}
static void probe_script(uint32_t bad)
{
    frame("0b 00 64 00 00 00 00 00", bad == 1 ? NULL : "00 00 00 00 21 43 65 87");
    if (bad == 1) return;
    frame("0b 00 74 00 00 00 00 00", "00 00 00 00 00 00 00 08");
    frame("0b 00 50 00 00 00 00 00", bad == 2 ? "00 00 00 00 01 00 53 92" : "00 00 00 00 01 00 52 92");
    if (bad == 2) return;
    csr_read("02 03 04 40 01 01 c0", bad == 3 ? "00 00 00 00 04 00 00 00" : "00 00 00 00 80 00 00 00");
    if (bad == 3) return;
    csr_read("02 03 04 02 05 02 c0", bad == 4 ? "00 00 00 00 00 08 00 00" :
             bad == 5 ? "00 00 00 00 00 10 00 00" :
             bad == 6 ? "00 00 00 00 00 20 00 00" :
             bad == 7 ? "00 00 00 00 00 40 00 00" :
             bad == 8 ? "00 00 00 00 00 80 00 00" : "00 00 00 00 80 00 00 00");
    if (bad >= 4) return;
    csr_read("02 03 04 00 00 04 c0", "00 00 00 00 c8 01 34 12");
    csr_read("02 03 04 30 01 02 c0", "00 00 00 00 08 00 00 00");
}
static void test_probe(void)
{
    static const LAN9252Status wants[] = {LAN9252_OK, LAN9252_ERR_BYTE_TEST,
        LAN9252_ERR_CHIP_ID, LAN9252_ERR_PDI, LAN9252_ERR_EEPROM,
        LAN9252_ERR_EEPROM, LAN9252_ERR_EEPROM, LAN9252_ERR_EEPROM, LAN9252_ERR_EEPROM};
    LAN9252ProbeResult p; unsigned i;
    for (i=0;i<9;++i) {
        reset(); probe_script(i); memset(&p, 0xff, sizeof(p));
        result(LAN9252_Probe(&ctx,&p),wants[i],0);
        CHECK(f.current == f.count);
        if (!i) {
            CHECK(p.byte_test == 0x87654321 && p.hw_cfg == 0x08000000 && p.id_rev == 0x92520001);
            CHECK(p.pdi_control == 0x80 && p.eeprom_status == 0x0080);
            CHECK(p.esc_type == 0xc8 && p.esc_revision == 1 && p.esc_build == 0x1234 && p.al_status == 8);
        }
    }
    reset(); frame("0b 00 64 00 00 00 00 00","00 00 00 00 21 43 65 87");
    frame("0b 00 74 00 00 00 00 00",NULL); f.repeat_last=1; f.step=100;
    f.now=UINT32_MAX-200; f.max_calls=10000;
    result(LAN9252_Probe(&ctx,&p),LAN9252_ERR_READY_TIMEOUT,1);
}
static void test_timeouts(void)
{
    uint8_t b[8] = {0}; int write, phase;
    for (write=0;write<2;++write) for (phase=0;phase<2;++phase) {
        reset(); f.now=UINT32_MAX-100; f.step=20; f.repeat_last=1; f.max_calls=10000;
        if (phase) {
            csr_idle();
            if (write) frame("02 03 00 00 00 00 00",NULL);
            frame(write ? "02 03 04 00 02 04 80" : "02 03 04 00 02 04 c0",NULL);
        }
        frame("0b 03 04 00 00 00 00 00","00 00 00 00 00 00 00 80");
        result(write ? LAN9252_WriteEsc(&ctx,0x200,b,4) : LAN9252_ReadEsc(&ctx,0x200,b,4),LAN9252_ERR_CSR_TIMEOUT,1);
    }
    for (write=0;write<2;++write) for (phase=0;phase<3;++phase) {
        const char *poll=write ? "0b 03 14 00 00 00 00 00" : "0b 03 0c 00 00 00 00 00";
        reset(); f.now=UINT32_MAX-100; f.step=20; f.repeat_last=1; f.max_calls=10000;
        if (!phase) frame(write ? "02 03 14 00 00 00 40" : "02 03 0c 00 00 00 40",NULL);
        else {
            pram_start(write,&pram_cases[0]);
            if (phase==2) {
                frame(poll,"00 00 00 00 01 01 00 80");
                frame(write ? "02 00 20 00 00 00 00" : "0b 00 04 00 00 00 00 00",NULL);
            }
        }
        frame(poll,"00 00 00 00 01 00 00 80");
        result(write ? LAN9252_WriteEsc(&ctx,0x1000,b,1) : LAN9252_ReadEsc(&ctx,0x1000,b,1),LAN9252_ERR_PRAM_TIMEOUT,1);
    }
}
static LAN9252Status operation(unsigned op, uint8_t *b, LAN9252ProbeResult *p)
{
    switch(op) {
    case 0:return LAN9252_ReadDirect(&ctx,0x1234,b,4);
    case 1:return LAN9252_WriteDirect(&ctx,0x1234,b,4);
    case 2:return LAN9252_ReadEsc(&ctx,0x200,b,4);
    case 3:return LAN9252_WriteEsc(&ctx,0x200,b,4);
    case 4:return LAN9252_ReadEsc(&ctx,0x1400,b,6);
    case 5:return LAN9252_WriteEsc(&ctx,0x1400,b,6);
    default:return LAN9252_Probe(&ctx,p);
    }
}
static void operation_script(unsigned op)
{
    switch(op) {
    case 0:frame("0b 12 34 00 00 00 00 00",NULL);break;
    case 1:frame("02 12 34 11 22 33 44",NULL);break;
    case 2:csr_read("02 03 04 00 02 04 c0",NULL);break;
    case 3:csr_write("02 03 00 11 22 33 44","02 03 04 00 02 04 80");break;
    case 4:pram_script(0,&pram_cases[4]);break;
    case 5:pram_script(1,&pram_cases[4]);break;
    default:probe_script(0);break;
    }
}
static void test_transport_failures(void)
{
    unsigned op, n, calls, mode;
    for(op=0;op<7;++op) {
        uint8_t b[6]={0x11,0x22,0x33,0x44,0x55,0x66}; LAN9252ProbeResult p;
        reset(); operation_script(op); result(operation(op,b,&p),LAN9252_OK,0); calls=f.calls;
        for(mode=0;mode<3;++mode) for(n=1;n<=calls;++n) {
            memcpy(b,"\x11\x22\x33\x44\x55\x66",6);
            reset(); operation_script(op);
            if(mode==2) f.late_call=n;
            else {f.fail_call=n;f.failure=mode ? LAN9252_ERR_DIRECT_TIMEOUT : LAN9252_ERR_IO;}
            result(operation(op,b,&p),mode ? LAN9252_ERR_DIRECT_TIMEOUT : LAN9252_ERR_IO,mode ? 1 : 0);
            CHECK(f.calls==n); /* Fail fast, release CS even after partial header/data. */
        }
    }
}
static void test_arguments(void)
{
    uint8_t b[8]={0}; LAN9252Transport t; LAN9252ProbeResult p;
    reset(); result(LAN9252_ReadDirect(&ctx,0,b,0),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_WriteDirect(&ctx,0,NULL,4),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_ReadDirect(&ctx,0xffff,b,2),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_WriteDirect(&ctx,0,b,(size_t)-1),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_ReadEsc(&ctx,0xfff,b,2),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_WriteEsc(&ctx,0x1fff,b,2),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_ReadEsc(&ctx,0x2000,b,1),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_WriteEsc(&ctx,0x1000,NULL,1),LAN9252_ERR_ARGUMENT,0);
    result(LAN9252_Probe(&ctx,NULL),LAN9252_ERR_ARGUMENT,0);
    CHECK(LAN9252_ReadDirect(NULL,0,b,4)==LAN9252_ERR_ARGUMENT);
    CHECK(LAN9252_WriteEsc(NULL,0,b,1)==LAN9252_ERR_ARGUMENT);
    CHECK(LAN9252_Probe(NULL,&p)==LAN9252_ERR_ARGUMENT);
    CHECK(f.selects==0);
    t=ctx.transport;t.time_us=NULL;LAN9252_InitContext(&ctx,&t);
    result(LAN9252_ReadDirect(&ctx,0,b,4),LAN9252_ERR_ARGUMENT,0);
    LAN9252_InitContext(&ctx,NULL);
    result(LAN9252_ReadEsc(&ctx,0,b,1),LAN9252_ERR_ARGUMENT,0);
    LAN9252_InitContext(NULL,NULL);
}
static void test_delayed_progress_and_wrap(void)
{
    uint8_t b[8]={0}; LAN9252ProbeResult p;
    reset(); f.now=UINT32_MAX-10;
    frame("0b 03 04 00 00 00 00 00","00 00 00 00 00 00 00 80");
    csr_idle();
    frame("02 03 04 ff 0f 01 c0",NULL);
    frame("0b 03 04 00 00 00 00 00","00 00 00 00 00 00 00 80");
    csr_idle();frame("0b 03 00 00 00 00 00 00","00 00 00 00 5a 99 99 99");
    result(LAN9252_ReadEsc(&ctx,0xfff,b+1,1),LAN9252_OK,0);
    CHECK(b[0]==0 && b[1]==0x5a && b[2]==0);
    reset();
    /* Insert an initial not-ready read into an otherwise successful probe. */
    probe_script(0);
    memmove(&f.frames[2],&f.frames[1],(f.count-1)*sizeof(Frame));
    ++f.count;memset(f.frames[1].rx,0,sizeof(f.frames[1].rx));
    f.now=UINT32_MAX-10;
    result(LAN9252_Probe(&ctx,&p),LAN9252_OK,0);
    CHECK(p.hw_cfg==0x08000000);
    reset(); frame("0b 12 34 00 00 00 00 00","00 00 00 00 11 22 33 44");
    f.now=UINT32_MAX-2;
    result(LAN9252_ReadDirect(&ctx,0x1234,b,4),LAN9252_OK,0);
    CHECK(!memcmp(b,"\x11\x22\x33\x44",4));
}
static void test_more_than_fifo_capacity(void)
{
    unsigned i; int write;
    for(write=0;write<2;++write) {
        uint8_t b[70];
        const char *poll=write ? "0b 03 14 00 00 00 00 00" : "0b 03 0c 00 00 00 00 00";
        reset();memset(b,0xa5,sizeof(b));
        frame(write ? "02 03 14 00 00 00 40" : "02 03 0c 00 00 00 40",NULL);
        frame(poll,NULL);
        frame(write ? "02 03 10 00 10 44 00" : "02 03 08 00 10 44 00",NULL);
        frame(write ? "02 03 14 00 00 00 80" : "02 03 0c 00 00 00 80",NULL);
        for(i=0;i<17;++i) {
            /* Reserved bits must not be interpreted as the DWORD count. */
            frame(poll,"00 00 00 00 01 e0 00 80");
            frame(poll,"00 00 00 00 01 01 00 80");
            frame(write ? "02 00 20 a5 a5 a5 a5" : "0b 00 04 00 00 00 00 00",
                  write ? NULL : "00 00 00 00 11 22 33 44");
        }
        frame(poll,NULL);
        result(write ? LAN9252_WriteEsc(&ctx,0x1000,b+1,68) : LAN9252_ReadEsc(&ctx,0x1000,b+1,68),LAN9252_OK,0);
        CHECK(b[0]==0xa5 && b[69]==0xa5);
        if(!write) for(i=0;i<17;++i) CHECK(!memcmp(b+1+4*i,"\x11\x22\x33\x44",4));
    }
}
static void test_diagnostics_and_bad_transports(void)
{
    uint8_t b[4]={0}; LAN9252Transport t; unsigned missing;
    for(missing=0;missing<4;++missing) {
        reset();t=ctx.transport;
        if(missing==0)t.transfer=NULL;
        if(missing==1)t.select=NULL;
        if(missing==2)t.deselect=NULL;
        if(missing==3)t.time_us=NULL;
        LAN9252_InitContext(&ctx,&t);
        result(LAN9252_WriteDirect(&ctx,0,b,4),LAN9252_ERR_ARGUMENT,0);
        CHECK(f.calls==0);
    }
    reset();frame("0b 00 64 00 00 00 00 00",NULL);
    f.fail_call=1;f.failure=LAN9252_ERR_DIRECT_TIMEOUT;
    result(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_ERR_DIRECT_TIMEOUT,1);
    frame("0b 00 64 00 00 00 00 00","00 00 00 00 21 43 65 87");
    f.fail_call=0;
    result(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_OK,1);
    CHECK(!memcmp(b,"\x21\x43\x65\x87",4));
    frame("0b 00 64 00 00 00 00 00",NULL);
    f.fail_call=f.calls+1;
    result(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_ERR_DIRECT_TIMEOUT,2);
    reset();frame("0b 00 64 00 00 00 00 00",NULL);
    f.fail_call=1;f.failure=LAN9252_ERR_BYTE_TEST;
    result(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_ERR_IO,0);
}
int main(void)
{
    test_direct_rejects_invalid_transactions();
    test_direct();test_csr();test_pram();test_probe();test_timeouts();
    test_transport_failures();test_arguments();
    test_delayed_progress_and_wrap();test_more_than_fifo_capacity();
    test_diagnostics_and_bad_transports();
    printf("test_lan9252_protocol: PASS (%u behavior cases)\n",cases);
    return 0;
}
