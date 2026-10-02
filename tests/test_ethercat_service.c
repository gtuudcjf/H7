#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ethercat_service.h"
#include "lan9252_port.h"
#include "ethercat_app_core.h"

static unsigned checks, init_calls, loop_calls, drains, releases, resets, reg_reads;
static uint32_t now, loop_us;
static uint16_t init_result;
static uint8_t fault, init_fault, loop_fault, read_fault;
static LAN9252PortObservations observations;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "service line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
uint8_t HW_Init(void) { fault = observations.last_error != LAN9252_OK; return fault; }
void HW_Release(void) { CHECK(drains > releases); ++releases; }
uint8_t HW_AccessFault(void) { return fault; }
uint32_t HW_GetTimer(void) { return now; }
const LAN9252PortObservations *LAN9252_PortGetObservations(void) { return &observations; }
uint16_t MainInit(void) { ++init_calls; if (init_fault) { fault=1; observations.last_error=LAN9252_ERR_IO; } return init_result; }
void MainLoop(void) { ++loop_calls; now += loop_us; if (loop_fault) { fault=1; observations.last_error=LAN9252_ERR_PRAM_TIMEOUT; } }
void MBX_StopMailboxHandler(void) { ++drains; }
void APPL_AckErrorInd(uint16_t t) { (void)t; ++resets; EtherCAT_AppReset(); }
void HW_EscRead(uint8_t *p, uint16_t a, uint16_t n)
{
    ++reg_reads; memset(p,0,n);
    if (read_fault) { fault=1; observations.last_error=LAN9252_ERR_CSR_TIMEOUT; return; }
    switch (a) {
    case 0x130: CHECK(n==2); p[0]=8; break;
    case 0x134: CHECK(n==2); p[0]=0x34; p[1]=0x12; break;
    case 0x220: CHECK(n==2); p[0]=0x78; p[1]=0x56; break;
    case 0x30d: CHECK(n==1); p[0]=9; break;
    case 0x310: CHECK(n==3); p[0]=2; p[1]=3; p[2]=4; break;
    default: CHECK(0);
    }
}
int main(void)
{
    const EtherCATDiagnostics *d=EtherCAT_GetDiagnostics(); uint8_t rx[2]={1,0}; unsigned loops, reads_before;
    CHECK(d->last_driver_error == LAN9252_OK); EtherCAT_Service(); CHECK(loop_calls==0);
    observations.probe.byte_test=0x87654321; observations.probe.id_rev=0x92520001;
    observations.probe.hw_cfg=0x8000000; observations.probe.pdi_control=0x80; observations.probe.eeprom_status=0x123;
    observations.last_error=LAN9252_ERR_PDI;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_LAN9252_ERROR && init_calls==0);
    CHECK(d->byte_test==0x87654321 && d->id_rev==0x92520001 && d->last_driver_error==LAN9252_ERR_PDI);
    observations.last_error=LAN9252_OK; init_result=7;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_SSC_ERROR); EtherCAT_Service(); CHECK(loop_calls==0);
    init_result=0; init_fault=1;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_LAN9252_ERROR && d->last_driver_error==LAN9252_ERR_IO);
    init_fault=0; observations.last_error=LAN9252_OK;
    CHECK(EtherCAT_Init()==ETHERCAT_STATUS_READY && d->pdi_control==0x80 && d->eeprom_status==0x123);
    EtherCAT_AppConsumeRx(rx); EtherCAT_AppCycle(); now=100000; loop_us=40;
    EtherCAT_Service(); CHECK(loop_calls==1 && d->ethercat_service_max_us==40);
    CHECK(d->al_status==8 && d->al_status_code==0x1234 && d->al_event==0x5678);
    CHECK(d->pdi_error_count==9 && d->lost_link_count==9);
    CHECK(d->rx_control_word==1 && d->tx_status_word==1 && d->tx_cycle_counter==1);
    reads_before=reg_reads; loop_us=10; EtherCAT_Service(); CHECK(reg_reads==reads_before && d->ethercat_service_max_us==40);
    now=0xfffffff0; loop_us=64; EtherCAT_Service(); CHECK(d->ethercat_service_max_us==64);
    loops=loop_calls; fault=1; observations.last_error=LAN9252_ERR_IO;
    EtherCAT_Service(); CHECK(loop_calls==loops && d->rx_control_word==0 && d->tx_cycle_counter==0 && d->last_driver_error==LAN9252_ERR_IO);
    fault=0; EtherCAT_Service(); CHECK(loop_calls==loops);
    observations.last_error=LAN9252_OK; CHECK(EtherCAT_Init()==ETHERCAT_STATUS_READY);
    loop_fault=1; EtherCAT_AppConsumeRx(rx); EtherCAT_AppCycle(); EtherCAT_Service();
    CHECK(d->last_driver_error==LAN9252_ERR_PRAM_TIMEOUT && d->tx_status_word==0 && d->tx_cycle_counter==0);
    loops=loop_calls; EtherCAT_Service(); CHECK(loop_calls==loops);
    loop_fault=0; observations.last_error=LAN9252_OK; CHECK(EtherCAT_Init()==ETHERCAT_STATUS_READY);
    read_fault=1; now+=100000; reads_before=reg_reads; EtherCAT_Service();
    CHECK(d->last_driver_error==LAN9252_ERR_CSR_TIMEOUT && reg_reads==reads_before+1);
    CHECK(resets > 0 && releases==drains);
    printf("EtherCAT service: %u checks passed\n",checks); return 0;
}
