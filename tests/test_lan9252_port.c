#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lan9252_port.h"
#include "lan9252_board.h"
#include "el9800hw.h"
#include "ethercat_service.h"
#include "ethercat_app_core.h"

static unsigned checks, probes, reads, writes;
static uint32_t now;
static LAN9252Status probe_error, access_error;
static LAN9252Context *seen_context;
static unsigned main_inits, main_loops, mailbox_drains;
static uint16_t main_init_error;
static uint8_t main_init_fault, main_loop_fault;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "port line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint32_t time_us(void *p) { (void)p; return now; }
static const LAN9252Transport transport = {NULL, NULL, NULL, time_us, NULL};
static unsigned transport_missing;
const LAN9252Transport *LAN9252_BoardTransport(void) { return transport_missing ? NULL : &transport; }
void LAN9252_InitContext(LAN9252Context *c, const LAN9252Transport *t)
{ memset(c, 0, sizeof(*c)); if(t)c->transport = *t; else c->last_error=LAN9252_ERR_ARGUMENT; seen_context = c; }
LAN9252Status LAN9252_Probe(LAN9252Context *c, LAN9252ProbeResult *r)
{ CHECK(c == seen_context); ++probes; memset(r, 0, sizeof(*r)); r->byte_test = 0x87654321; r->id_rev = 0x92520001; c->last_error = probe_error; return probe_error; }
LAN9252Status LAN9252_ReadEsc(LAN9252Context *c, uint16_t a, uint8_t *p, size_t n)
{ CHECK(c == seen_context); ++reads; if (p) memset(p, 0xa5, n); if (a == 0x220 && p && n == 2) { p[0]=0x34; p[1]=0x12; } c->last_error = access_error; if (access_error == LAN9252_ERR_DIRECT_TIMEOUT) ++c->timeout_count; return access_error; }
LAN9252Status LAN9252_WriteEsc(LAN9252Context *c, uint16_t a, const uint8_t *p, size_t n)
{ (void)a; (void)p; (void)n; CHECK(c == seen_context); ++writes; c->last_error = access_error; return access_error; }
uint16_t MainInit(void)
{
    uint8_t data; ++main_inits;
    if (main_init_fault) { access_error=LAN9252_ERR_IO; HW_EscRead(&data,0x140,1); }
    return main_init_error;
}
void MainLoop(void)
{
    uint8_t data; ++main_loops;
    if (main_loop_fault) { access_error=LAN9252_ERR_PRAM_TIMEOUT; HW_EscRead(&data,0x140,1); }
}
void MBX_StopMailboxHandler(void) { ++mailbox_drains; }
void APPL_AckErrorInd(uint16_t transition) { (void)transition; EtherCAT_AppReset(); }

int main(void)
{
    uint8_t bytes[8]; void *blocks[24]; unsigned i; uint32_t *aligned;
    transport_missing=1;
    CHECK(HW_Init()!=0 && probes==0 && HW_AccessFault());
    CHECK(LAN9252_PortGetObservations()->last_error==LAN9252_ERR_CLOCK);
    transport_missing=0;
    probe_error = LAN9252_ERR_PDI;
    CHECK(HW_Init() != 0 && probes == 1 && HW_AccessFault());
    CHECK(LAN9252_PortGetObservations()->probe.byte_test == 0x87654321);
    CHECK(LAN9252_PortGetObservations()->last_error == LAN9252_ERR_PDI);
    probe_error = LAN9252_OK; CHECK(HW_Init() == 0 && probes == 2 && !HW_AccessFault());
    now = 0xfffffff0; CHECK(HW_GetTimer() == 0xfffffff0); now = 0x20; CHECK(HW_GetTimer() == 0x20);
    CHECK(HW_GetALEventRegister() == 0x1234);
    memset(bytes, 0xcc, sizeof(bytes)); access_error = LAN9252_ERR_DIRECT_TIMEOUT;
    HW_EscRead(bytes + 1, 0x1001, 5);
    CHECK(bytes[0] == 0xcc && bytes[6] == 0xcc && bytes[7] == 0xcc);
    for (i=1;i<6;i++) CHECK(bytes[i] == 0);
    CHECK(HW_AccessFault() && LAN9252_PortGetObservations()->last_error == LAN9252_ERR_DIRECT_TIMEOUT);
    access_error = LAN9252_OK; HW_EscRead(bytes + 1, 0x1001, 5); HW_EscWrite(bytes + 1, 0x1001, 5);
    CHECK(bytes[1] == 0xa5 && writes == 1 && HW_AccessFault());
    CHECK(seen_context->last_error == LAN9252_OK);
    CHECK(LAN9252_PortGetObservations()->last_error == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(LAN9252_PortGetObservations()->timeout_count == 1);
    HW_RecordSyncManTimeout(3);
    CHECK(LAN9252_PortGetObservations()->last_error == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(HW_Init() == 0); HW_RecordSyncManTimeout(2);
    CHECK(HW_AccessFault() && LAN9252_PortGetObservations()->last_error == LAN9252_ERR_SM_TIMEOUT);
    CHECK(LAN9252_PortGetObservations()->syncman_timeout_channel == 2);
    CHECK(HW_Init() == 0); HW_EscRead(NULL, 0, 1); CHECK(HW_AccessFault());
    CHECK(LAN9252_PortGetObservations()->last_error == LAN9252_ERR_ARGUMENT);
    CHECK(HW_Init() == 0); HW_EscRead(bytes, 0, 0); CHECK(HW_AccessFault());
    CHECK(HW_AllocMailboxBuffer(0) == NULL && HW_AllocMailboxBuffer(9) == NULL && HW_AllocMailboxBuffer(129) == NULL);
    for (i=0;i<24;i++) { blocks[i]=HW_AllocMailboxBuffer(i==0 ? 10 : 128); CHECK(blocks[i] != NULL); CHECK(((uintptr_t)blocks[i] & 3)==0); memset(blocks[i], (int)i, 128); }
    CHECK(HW_AllocMailboxBuffer(128) == NULL);
    HW_FreeMailboxBuffer(NULL); HW_FreeMailboxBuffer(bytes); HW_FreeMailboxBuffer((uint8_t *)blocks[0]+1);
    CHECK(HW_AllocMailboxBuffer(128) == NULL);
    CHECK(HW_Init() == 0); CHECK(HW_AllocMailboxBuffer(128) == NULL);
    aligned = blocks[23]; CHECK(((uint8_t *)aligned)[127] == 23);
    HW_FreeMailboxBuffer(blocks[12]); CHECK(HW_AllocMailboxBuffer(128) == blocks[12]);
    for (i=0;i<24;i++) HW_FreeMailboxBuffer(blocks[i]);
    HW_FreeMailboxBuffer(blocks[0]);
    for (i=0;i<24;i++) CHECK(HW_AllocMailboxBuffer(128) != NULL);
    HW_Release(); CHECK(HW_AllocMailboxBuffer(128) != NULL); HW_Release();
    probe_error=LAN9252_ERR_BYTE_TEST; i=probes;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_LAN9252_ERROR && probes==i+1 && main_inits==0);
    CHECK(EtherCAT_GetDiagnostics()->byte_test==0x87654321);
    probe_error=LAN9252_OK; access_error=LAN9252_OK; i=probes;
    main_init_fault=1; main_init_error=7;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_LAN9252_ERROR && probes==i+1 && main_inits==1);
    CHECK(EtherCAT_GetDiagnostics()->last_driver_error==LAN9252_ERR_IO);
    main_init_fault=0; main_init_error=0; access_error=LAN9252_OK; i=probes;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_READY && probes==i+1 && mailbox_drains==1);
    i=probes; now+=100000; EtherCAT_Service();
    CHECK(probes==i && main_loops==1 && EtherCAT_GetDiagnostics()->al_status==0xa5a5);
    access_error=LAN9252_ERR_DIRECT_TIMEOUT; now+=100000; i=reads; EtherCAT_Service();
    CHECK(reads==i+1 && EtherCAT_GetDiagnostics()->last_driver_error==LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(EtherCAT_GetDiagnostics()->hbi_timeout_count==1);
    CHECK(EtherCAT_GetDiagnostics()->al_status==0xa5a5 && EtherCAT_GetDiagnostics()->byte_test==0x87654321);
    access_error=LAN9252_OK; HW_EscRead(bytes,0x130,2);
    CHECK(HW_AccessFault() && LAN9252_PortGetObservations()->last_error==LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(seen_context->last_error==LAN9252_OK);
    i=main_loops; EtherCAT_Service(); CHECK(main_loops==i);
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_READY && mailbox_drains==2);
    main_loop_fault=1; EtherCAT_Service();
    CHECK(EtherCAT_GetDiagnostics()->last_driver_error==LAN9252_ERR_PRAM_TIMEOUT);
    printf("LAN9252 port: %u checks passed (probes=%u reads=%u writes=%u)\n", checks, probes, reads, writes); return 0;
}
