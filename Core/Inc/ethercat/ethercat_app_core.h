#ifndef ETHERCAT_APP_CORE_H
#define ETHERCAT_APP_CORE_H

#include <stdint.h>

typedef struct { uint16_t control_word; } EtherCATRxPdo;
typedef struct { uint16_t status_word; uint32_t cycle_counter; } EtherCATTxPdo;

void EtherCAT_AppReset(void);
void EtherCAT_AppConsumeRx(const uint8_t rx[2]);
void EtherCAT_AppProduceTx(uint8_t tx[6]);
void EtherCAT_AppCycle(void);
const EtherCATRxPdo *EtherCAT_AppGetRx(void);
const EtherCATTxPdo *EtherCAT_AppGetTx(void);

#endif
