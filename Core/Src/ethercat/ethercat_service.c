#include "ethercat_service.h"
#include "lan9252_port.h"
#include "el9800hw.h"
#include "applInterface.h"
#include "ethercat_app.h"
#include "ethercat_app_core.h"
#include "mailbox.h"
#include <string.h>

#define DIAGNOSTICS_INTERVAL_US UINT32_C(100000)
static EtherCATStatus status;
static EtherCATDiagnostics diagnostics;
static uint32_t last_diagnostics_us;
static uint8_t stack_started;

static void ObservePort(void)
{
    const LAN9252PortObservations *p = LAN9252_PortGetObservations();
    diagnostics.byte_test = p->probe.byte_test;
    diagnostics.id_rev = p->probe.id_rev;
    diagnostics.hw_cfg = p->probe.hw_cfg;
    diagnostics.pdi_control = p->probe.pdi_control;
    diagnostics.eeprom_status = p->probe.eeprom_status;
    diagnostics.last_driver_error = p->last_error;
    diagnostics.spi_timeout_count = p->timeout_count;
}

static void ObserveProcessData(void)
{
    diagnostics.rx_control_word = EtherCAT_AppGetRx()->control_word;
    diagnostics.tx_status_word = EtherCAT_AppGetTx()->status_word;
    diagnostics.tx_cycle_counter = EtherCAT_AppGetTx()->cycle_counter;
}

static void StopOnFault(void)
{
    status = ETHERCAT_STATUS_LAN9252_ERROR;
    APPL_AckErrorInd(0);
    ObserveProcessData();
    ObservePort();
}

EtherCATStatus EtherCAT_Init(void)
{
    UINT16 result;
    if (stack_started) {
        MBX_StopMailboxHandler();
        HW_Release();
        stack_started = 0;
    }
    APPL_AckErrorInd(0);
    memset(&diagnostics, 0, sizeof(diagnostics));
    status = ETHERCAT_STATUS_UNINITIALIZED;
    result = HW_Init();
    ObservePort();
    diagnostics.al_status = LAN9252_PortGetObservations()->probe.al_status;
    if (result || HW_AccessFault()) {
        StopOnFault();
        return status;
    }
    stack_started = 1; /* MainInit may create retained ownership before failing. */
    result = MainInit();
    ObservePort();
    if (HW_AccessFault()) StopOnFault();
    else if (result) {
        status = ETHERCAT_STATUS_SSC_ERROR;
        APPL_AckErrorInd(0);
    } else status = ETHERCAT_STATUS_READY;
    last_diagnostics_us = HW_GetTimer();
    return status;
}

static int ReadWord(uint16_t address, uint16_t *value)
{
    uint8_t bytes[2];
    HW_EscRead(bytes, address, 2);
    if (HW_AccessFault()) return 0;
    *value = (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    return 1;
}

static void UpdateDiagnostics(void)
{
    uint8_t bytes[3];
    if (!ReadWord(0x0130, &diagnostics.al_status)) return;
    if (!ReadWord(0x0134, &diagnostics.al_status_code)) return;
    if (!ReadWord(0x0220, &diagnostics.al_event)) return;
    HW_EscRead(bytes, 0x030d, 1);
    if (HW_AccessFault()) return;
    diagnostics.pdi_error_count = bytes[0];
    /* DS00001909A: byte counters for ports 0..2; port 3 is unused.
     * Sum includes port 2 (zero when unused on this two-PHY board). */
    HW_EscRead(bytes, 0x0310, 3);
    if (HW_AccessFault()) return;
    diagnostics.lost_link_count = (uint32_t)bytes[0] + bytes[1] + bytes[2];
    ObserveProcessData();
}

void EtherCAT_Service(void)
{
    uint32_t start, elapsed;
    if (status != ETHERCAT_STATUS_READY) return;
    start = HW_GetTimer();
    if (HW_AccessFault()) StopOnFault();
    else {
        MainLoop(); /* SSC owns the monotonic timer baseline and bounded catch-up. */
        if (HW_AccessFault()) StopOnFault();
        else if ((uint32_t)(HW_GetTimer() - last_diagnostics_us) >= DIAGNOSTICS_INTERVAL_US) {
            last_diagnostics_us = HW_GetTimer();
            UpdateDiagnostics();
            if (HW_AccessFault()) StopOnFault();
        }
    }
    ObservePort();
    elapsed = (uint32_t)(HW_GetTimer() - start);
    if (elapsed > diagnostics.ethercat_service_max_us)
        diagnostics.ethercat_service_max_us = elapsed;
}

const EtherCATDiagnostics *EtherCAT_GetDiagnostics(void) { return &diagnostics; }
