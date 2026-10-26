#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lan9252_hbi.h"

/* Fixtures describe physical halfword transactions, independently of the
 * production CSR/PRAM algorithms. Wrong shifts, halfword order, partial DWORD
 * reads, FIFO over-consumption and timeout fallthrough fail at this boundary. */
typedef struct { unsigned write; uint16_t address, value; } Word;
static Word script[8192];
static unsigned count, current, checks, halves, guards, fail_at, late_at;
static uint32_t clock_us, tick_step;
static LAN9252Context ctx;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"HBI line %d: %s (word %u/%u)\n",__LINE__,#x,current,count); exit(1); } } while (0)
static void pair(unsigned write, uint16_t address, uint32_t value)
{
    script[count].write=write; script[count].address=address;
    script[count++].value=(uint16_t)value;
    script[count].write=write; script[count].address=(uint16_t)(address+2);
    script[count++].value=(uint16_t)(value>>16);
    CHECK(count < 8190);
}
static LAN9252Status word(unsigned write, uint16_t a, uint16_t *v)
{
    Word *p;
    CHECK(current<count); p=&script[current++];
    CHECK(p->write==write && p->address==a);
    if (write) CHECK(*v==p->value); else *v=p->value;
    ++halves;
    if (late_at==current) clock_us+=1001;
    if (fail_at==current) return LAN9252_ERR_IO;
    return LAN9252_OK;
}
static LAN9252Status read16(void *p,uint16_t a,uint16_t *v) { (void)p; return word(0,a,v); }
static LAN9252Status write16(void *p,uint16_t a,uint16_t v) { (void)p; return word(1,a,&v); }
static void guard(void *p) { (void)p; ++guards; ++clock_us; }
static uint32_t now(void *p) { uint32_t v=clock_us; (void)p; clock_us+=tick_step; return v; }
static void reset(void)
{
    LAN9252Transport t={read16,write16,guard,now,NULL};
    count=current=halves=guards=fail_at=late_at=0;
    clock_us=0; tick_step=1;
    LAN9252_InitContext(&ctx,&t);
}
static void done(LAN9252Status got,LAN9252Status want)
{
    CHECK(got==want); CHECK(current==count);
    CHECK(ctx.last_error==want);
}
static void csr_read(uint16_t a,unsigned n,uint32_t v)
{
    pair(0,0x304,0); pair(1,0x304,0xc0000000U|((uint32_t)n<<16)|a);
    pair(0,0x304,0); pair(0,0x300,v);
}
static void csr_write(uint16_t a,unsigned n,uint32_t v)
{
    pair(0,0x304,0); pair(1,0x300,v);
    pair(1,0x304,0x80000000U|((uint32_t)n<<16)|a); pair(0,0x304,0);
}
static void test_direct_and_arguments(void)
{
    uint8_t b[8]={0xaa}; unsigned i;
    reset(); pair(0,0x64,0x87654321);
    done(LAN9252_ReadDirect(&ctx,0x64,b+1,4),LAN9252_OK);
    CHECK(!memcmp(b,"\xaa\x21\x43\x65\x87\0",6)); CHECK(halves==2 && guards==1);
    pair(1,0x3fc,0x87654321);
    done(LAN9252_WriteDirect(&ctx,0x3fc,b+1,4),LAN9252_OK);
    CHECK(guards==2);
    for(i=0;i<9;++i) if(i!=4) {
        reset(); done(LAN9252_ReadDirect(&ctx,0x64,b,i),LAN9252_ERR_ARGUMENT);
    }
    reset(); done(LAN9252_ReadDirect(&ctx,0x65,b,4),LAN9252_ERR_ARGUMENT);
    done(LAN9252_WriteDirect(&ctx,0x400,b,4),LAN9252_ERR_ARGUMENT);
    done(LAN9252_ReadEsc(&ctx,0xfff,b,2),LAN9252_ERR_ARGUMENT);
    done(LAN9252_WriteEsc(&ctx,0x1fff,b,2),LAN9252_ERR_ARGUMENT);
    done(LAN9252_ReadEsc(&ctx,0x2000,b,1),LAN9252_ERR_ARGUMENT);
    done(LAN9252_WriteEsc(&ctx,0x1000,NULL,1),LAN9252_ERR_ARGUMENT);
    CHECK(LAN9252_ReadEsc(NULL,0,b,1)==LAN9252_ERR_ARGUMENT);
    CHECK(halves==0 && guards==0);
    for(i=0;i<4;++i) {
        LAN9252Transport t; reset(); t=ctx.transport;
        if(i==0)t.read16=NULL;
        if(i==1)t.write16=NULL;
        if(i==2)t.finish_dword=NULL;
        if(i==3)t.time_us=NULL;
        LAN9252_InitContext(&ctx,&t);
        done(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_ERR_ARGUMENT);
    }
}
static void test_csr_alignment(void)
{
    uint8_t b[8]={0xee,0,0,0,0,0,0,0xdd};
    reset(); csr_read(0x121,1,0x11); csr_read(0x122,2,0x3322);
    csr_read(0x124,2,0x5544); csr_read(0x126,1,0x66);
    done(LAN9252_ReadEsc(&ctx,0x121,b+1,6),LAN9252_OK);
    CHECK(!memcmp(b,"\xee\x11\x22\x33\x44\x55\x66\xdd",8));
    reset(); csr_write(0x121,1,0x11); csr_write(0x122,2,0x3322);
    csr_write(0x124,2,0x5544); csr_write(0x126,1,0x66);
    done(LAN9252_WriteEsc(&ctx,0x121,b+1,6),LAN9252_OK);
    reset(); csr_read(0x200,4,0x78563412);
    done(LAN9252_ReadEsc(&ctx,0x200,b+1,4),LAN9252_OK);
    CHECK(!memcmp(b+1,"\x12\x34\x56\x78",4));
}
static void pram_start(unsigned write,uint16_t address,unsigned n)
{
    pair(1,write?0x314:0x30c,0x40000000);
    pair(0,write?0x314:0x30c,0);
    pair(1,write?0x310:0x308,((uint32_t)n<<16)|address);
    pair(1,write?0x314:0x30c,0x80000000);
}
static void test_pram(void)
{
    unsigned write,offset,n,i;
    for(write=0;write<2;++write) for(offset=0;offset<4;++offset) for(n=1;n<=130;++n) {
        uint8_t b[134]; unsigned remaining=n,pos=offset,index=0;
        uint16_t cmd=(uint16_t)(write?0x314:0x30c);
        reset(); memset(b,0xa5,sizeof(b)); pram_start(write,(uint16_t)(0x1000+offset),n);
        while(remaining) {
            unsigned take=remaining<4-pos?remaining:4-pos;
            uint32_t value=0;
            /* AVAIL without count and count without AVAIL must both wait. */
            pair(0,cmd,0x80000001); pair(0,cmd,0x80000100); pair(0,cmd,0x80000101);
            for(i=0;i<take;++i) value|=(uint32_t)(write?0xa5:(index+i+1))<<(8*(pos+i));
            pair(write,(uint16_t)(write?0x20:0x04),value);
            index+=take; remaining-=take; pos=0;
        }
        pair(0,cmd,0);
        done(write?LAN9252_WriteEsc(&ctx,(uint16_t)(0x1000+offset),b+1,n):
                   LAN9252_ReadEsc(&ctx,(uint16_t)(0x1000+offset),b+1,n),LAN9252_OK);
        CHECK(b[0]==0xa5 && b[n+1]==0xa5);
        for(i=0;i<n;++i) CHECK(b[i+1]==(write?0xa5:i+1));
    }
}
static void probe_rest(uint8_t pdi,uint16_t eeprom)
{
    pair(0,0x74,0x08000000); pair(0,0x50,0x92520001);
    csr_read(0x140,1,pdi); if(pdi!=0x89)return;
    csr_read(0x502,2,eeprom); if(eeprom&0xf800)return;
    csr_read(0,4,0x123401c8); csr_read(0x130,2,8);
}
static void test_probe(void)
{
    LAN9252ProbeResult p;
    reset(); pair(0,0x64,0xffffffff); pair(0,0x64,0x87654321); probe_rest(0x89,0x80);
    done(LAN9252_Probe(&ctx,&p),LAN9252_OK);
    CHECK(p.byte_test==0x87654321 && p.pdi_control==0x89 && p.al_status==8);
    CHECK(p.esc_type==0xc8 && p.esc_build==0x1234 && p.id_rev==0x92520001);
    reset(); pair(0,0x64,0x87654321); probe_rest(0x80,0);
    done(LAN9252_Probe(&ctx,&p),LAN9252_ERR_PDI);
    reset(); pair(0,0x64,0x87654321); probe_rest(0x89,0x800);
    done(LAN9252_Probe(&ctx,&p),LAN9252_ERR_EEPROM);
}
static void test_deadlines_and_faults(void)
{
    uint8_t b[4]={0}; unsigned half;
    for(half=1;half<=2;++half) {
        reset(); memset((void*)&g_lan9252_first_fault,0,sizeof(g_lan9252_first_fault));
        pair(0,0x304,0); count=half; fail_at=half;
        done(LAN9252_ReadEsc(&ctx,0x220,b,2),LAN9252_ERR_IO);
        CHECK(g_lan9252_first_fault.valid && g_lan9252_first_fault.phase==half);
        CHECK(g_lan9252_first_fault.address==0x304 && g_lan9252_first_fault.esc_address==0x220);
        CHECK(guards==1);
        reset(); pair(0,0x64,0x87654321);
        done(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_OK);
        CHECK(g_lan9252_first_fault.address==0x304); /* sticky first fault */
    }
    reset(); pair(0,0x64,0x87654321); late_at=2;
    done(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(ctx.timeout_count==1);
    reset(); clock_us=0xfffffffc; pair(0,0x64,0x87654321);
    done(LAN9252_ReadDirect(&ctx,0x64,b,4),LAN9252_OK);
    reset(); tick_step=100; clock_us=0xfffffff0;
    /* Explicit finite busy script, enough to expire the enclosing CSR budget. */
    for(half=0;half<40;++half) pair(0,0x304,0x80000000);
    CHECK(LAN9252_ReadEsc(&ctx,0x220,b,2)==LAN9252_ERR_CSR_TIMEOUT);
    CHECK(current<=count && ctx.timeout_count==1);
}
static void test_probe_and_pram_timeouts(void)
{
    unsigned i,write,phase;
    uint8_t b[4]={0};
    LAN9252ProbeResult p;
    reset(); tick_step=100;
    for(i=0;i<300;++i) pair(0,0x64,0xffffffff);
    CHECK(LAN9252_Probe(&ctx,&p)==LAN9252_ERR_BYTE_TEST);
    CHECK(current<count && p.byte_test==0xffffffff && p.id_rev==0);
    reset(); tick_step=100; clock_us=0xffff0000;
    pair(0,0x64,0x87654321);
    for(i=0;i<300;++i) pair(0,0x74,0);
    CHECK(LAN9252_Probe(&ctx,&p)==LAN9252_ERR_READY_TIMEOUT);
    CHECK(current<count && ctx.timeout_count==1);
    for(write=0;write<2;++write) for(phase=0;phase<3;++phase) {
        uint16_t cmd=(uint16_t)(write?0x314:0x30c);
        reset(); tick_step=100; clock_us=0xffffff00;
        if(phase==0) pair(1,cmd,0x40000000);
        else {
            pram_start(write,0x1000,4);
            if(phase==2) {
                pair(0,cmd,0x80000101);
                pair(write,(uint16_t)(write?0x20:0x04),0);
            }
        }
        for(i=0;i<40;++i) pair(0,cmd,0x80000000);
        CHECK((write?LAN9252_WriteEsc(&ctx,0x1000,b,4):LAN9252_ReadEsc(&ctx,0x1000,b,4))==LAN9252_ERR_PRAM_TIMEOUT);
        CHECK(current<=count && ctx.timeout_count==1);
    }
}
int main(void)
{
    test_direct_and_arguments(); test_csr_alignment(); test_pram(); test_probe(); test_deadlines_and_faults();
    test_probe_and_pram_timeouts();
    printf("test_lan9252_hbi: PASS (%u checks)\n",checks); return 0;
}
