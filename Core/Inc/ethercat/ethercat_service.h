#ifndef ETHERCAT_SERVICE_H
#define ETHERCAT_SERVICE_H
#include "lan9252_spi.h"

typedef enum {
    ETHERCAT_STATUS_UNINITIALIZED = 0,
    ETHERCAT_STATUS_READY,
    ETHERCAT_STATUS_LAN9252_ERROR,
    ETHERCAT_STATUS_SSC_ERROR
} EtherCATStatus;

typedef struct {
    uint32_t byte_test, id_rev, hw_cfg;
    uint8_t pdi_control;
    uint16_t eeprom_status, al_status, al_status_code, al_event;
    uint32_t pdi_error_count, lost_link_count;
    LAN9252Status last_driver_error;
    uint32_t spi_timeout_count, ethercat_service_max_us;
    uint16_t rx_control_word, tx_status_word;
    uint32_t tx_cycle_counter;
} EtherCATDiagnostics;

/* Foreground polling. Reinitialization explicitly drains the old SSC session.
 * Faults stop stack work until EtherCAT_Init is called again. */
EtherCATStatus EtherCAT_Init(void);
void EtherCAT_Service(void);
const EtherCATDiagnostics *EtherCAT_GetDiagnostics(void);
#endif
