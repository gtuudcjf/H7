#ifndef ETHERCAT_MOTOR_BRIDGE_H
#define ETHERCAT_MOTOR_BRIDGE_H
#include "ethercat_motor_core.h"
void EtherCATMotorBridge_Init(void);
void EtherCATMotorBridge_StartupFailed(void);
void EtherCATMotorBridge_SetLink(uint8_t active, uint8_t spi_ok);
void EtherCATMotorBridge_Receive(const uint8_t *wire, size_t size);
void EtherCATMotorBridge_Service(void);
void EtherCATMotorBridge_Pack(uint8_t wire[EMC_TX_BYTES]);
const EMC_Core *EtherCATMotorBridge_Get(void);
#endif
