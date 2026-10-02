#include "ethercat_app_core.h"

static EtherCATRxPdo rx_pdo;
static EtherCATTxPdo tx_pdo;

void EtherCAT_AppReset(void)
{
    rx_pdo.control_word = 0;
    tx_pdo.status_word = 0;
    tx_pdo.cycle_counter = 0;
}

void EtherCAT_AppConsumeRx(const uint8_t rx[2])
{
    rx_pdo.control_word = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8));
}

void EtherCAT_AppProduceTx(uint8_t tx[6])
{
    /* 线上固定六字节，不发送 C 结构的对齐空隙。 */
    tx[0] = (uint8_t)tx_pdo.status_word;
    tx[1] = (uint8_t)(tx_pdo.status_word >> 8);
    tx[2] = (uint8_t)tx_pdo.cycle_counter;
    tx[3] = (uint8_t)(tx_pdo.cycle_counter >> 8);
    tx[4] = (uint8_t)(tx_pdo.cycle_counter >> 16);
    tx[5] = (uint8_t)(tx_pdo.cycle_counter >> 24);
}

void EtherCAT_AppCycle(void)
{
    tx_pdo.status_word = rx_pdo.control_word & UINT16_C(1);
    tx_pdo.cycle_counter++;
}

const EtherCATRxPdo *EtherCAT_AppGetRx(void) { return &rx_pdo; }
const EtherCATTxPdo *EtherCAT_AppGetTx(void) { return &tx_pdo; }
