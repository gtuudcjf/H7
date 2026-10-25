/* Revision-2 application dictionary. Storage is DWORD aligned, wire is not. */
#include "ethercat_motor_bridge.h"
#include <string.h>
static struct { UINT16 count; UINT32 entries[3]; } rx_map_motor =
    {3,{0x70000110U,0x70000210U,0x70000320U}};
static struct { UINT16 count; UINT32 entries[8]; } tx_map_motor =
    {8,{0x60000110U,0x60000220U,0x60000310U,0x60000410U,
        0x60000510U,0x60000610U,0x60000710U,0x60000820U}};
static struct { UINT16 count, entry; } rx_assign_motor={1,0x1600}, tx_assign_motor={1,0x1a00};
static struct { UINT16 count,control; INT16 speed; UINT32 sequence; } motor_outputs={3,0,0,0};
static struct {
    UINT16 count,status; UINT32 counter; INT16 speed,iq;
    UINT16 motor_fault,bridge_fault; INT16 active; UINT32 ack;
} motor_inputs={8,0,0,0,0,0,0,0,0};
static const TSDOINFOENTRYDESC motor_map_desc[]={
    {DEFTYPE_UNSIGNED8,8,ACCESS_READ},{DEFTYPE_UNSIGNED32,32,ACCESS_READ}
};
static const TSDOINFOENTRYDESC motor_assign_desc[]={
    {DEFTYPE_UNSIGNED8,8,ACCESS_READ},{DEFTYPE_UNSIGNED16,16,ACCESS_READ}
};
static const TSDOINFOENTRYDESC motor_out_desc[]={
    {DEFTYPE_UNSIGNED8,8,ACCESS_READ},
    {DEFTYPE_UNSIGNED16,16,ACCESS_READ|OBJACCESS_RXPDOMAPPING},
    {DEFTYPE_INTEGER16,16,ACCESS_READ|OBJACCESS_RXPDOMAPPING},
    {DEFTYPE_UNSIGNED32,32,ACCESS_READ|OBJACCESS_RXPDOMAPPING}
};
static const TSDOINFOENTRYDESC motor_in_desc[]={
    {DEFTYPE_UNSIGNED8,8,ACCESS_READ},
    {DEFTYPE_UNSIGNED16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_UNSIGNED32,32,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_INTEGER16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_INTEGER16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_UNSIGNED16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_UNSIGNED16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_INTEGER16,16,ACCESS_READ|OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_UNSIGNED32,32,ACCESS_READ|OBJACCESS_TXPDOMAPPING}
};
static const UCHAR motor_rx_name[]="Motor RxPDO\000\377";
static const UCHAR motor_tx_name[]="Motor TxPDO\000\377";
static const UCHAR motor_rx_assign_name[]="RxPDO assign\000\377";
static const UCHAR motor_tx_assign_name[]="TxPDO assign\000\377";
static const UCHAR motor_in_name[]="Motor inputs\000StatusWord\000CycleCounter\000ActualSpeed\000ActualIq\000MotorFault\000BridgeFault\000ActiveTargetSpeed\000AcceptedSequence\000\377";
static const UCHAR motor_out_name[]="Motor outputs\000ControlWord\000TargetSpeed\000CommandSequence\000\377";
TOBJECT OBJMEM ApplicationObjDic[]={
    {NULL,NULL,0x1600,{DEFTYPE_PDOMAPPING,3|(OBJCODE_ARR<<8)},motor_map_desc,motor_rx_name,&rx_map_motor,NULL,NULL,0},
    {NULL,NULL,0x1a00,{DEFTYPE_PDOMAPPING,8|(OBJCODE_ARR<<8)},motor_map_desc,motor_tx_name,&tx_map_motor,NULL,NULL,0},
    {NULL,NULL,0x1c12,{DEFTYPE_UNSIGNED16,1|(OBJCODE_ARR<<8)},motor_assign_desc,motor_rx_assign_name,&rx_assign_motor,NULL,NULL,0},
    {NULL,NULL,0x1c13,{DEFTYPE_UNSIGNED16,1|(OBJCODE_ARR<<8)},motor_assign_desc,motor_tx_assign_name,&tx_assign_motor,NULL,NULL,0},
    {NULL,NULL,0x6000,{DEFTYPE_RECORD,8|(OBJCODE_REC<<8)},motor_in_desc,motor_in_name,&motor_inputs,NULL,NULL,0},
    {NULL,NULL,0x7000,{DEFTYPE_RECORD,3|(OBJCODE_REC<<8)},motor_out_desc,motor_out_name,&motor_outputs,NULL,NULL,0},
    {NULL,NULL,0xffff,{0,0},NULL,NULL,NULL,NULL,NULL,0}
};
static UINT16 MotorRead16(const UINT8 *b) { return (UINT16)(b[0]|((UINT16)b[1]<<8)); }
static UINT32 MotorRead32(const UINT8 *b) { return (UINT32)MotorRead16(b)|((UINT32)MotorRead16(b+2)<<16); }
static BOOL MotorExchangeActive(void)
{ return bEcatInputUpdateRunning && bEcatOutputUpdateRunning && bEcatFirstOutputsReceived && !HW_AccessFault(); }
static void MotorPublish(void)
{
    UINT8 b[EMC_TX_BYTES]; EtherCATMotorBridge_Pack(b);
    motor_inputs.status=MotorRead16(b); motor_inputs.counter=MotorRead32(b+2);
    motor_inputs.speed=(INT16)MotorRead16(b+6); motor_inputs.iq=(INT16)MotorRead16(b+8);
    motor_inputs.motor_fault=MotorRead16(b+10); motor_inputs.bridge_fault=MotorRead16(b+12);
    motor_inputs.active=(INT16)MotorRead16(b+14); motor_inputs.ack=MotorRead32(b+16);
}
static void MotorExchangeStop(void)
{
    EtherCATMotorBridge_SetLink(0,!HW_AccessFault());
    motor_outputs.control=0; motor_outputs.speed=0; motor_outputs.sequence=0;
    MotorPublish();
}
void APPL_AckErrorInd(UINT16 stateTrans) { (void)stateTrans; MotorExchangeStop(); }
UINT16 APPL_StartMailboxHandler(void) { MotorExchangeStop(); return 0; }
UINT16 APPL_StopMailboxHandler(void) { MotorExchangeStop(); return 0; }
UINT16 APPL_StartInputHandler(UINT16 *pIntMask) { (void)pIntMask; MotorExchangeStop(); return 0; }
UINT16 APPL_StopInputHandler(void) { MotorExchangeStop(); return 0; }
UINT16 APPL_StartOutputHandler(void) { MotorExchangeStop(); return 0; }
UINT16 APPL_StopOutputHandler(void) { MotorExchangeStop(); return 0; }
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{ *pInputSize=EMC_TX_BYTES; *pOutputSize=EMC_RX_BYTES; return 0; }
void APPL_OutputMapping(UINT16 *pData)
{
    UINT8 *b=(UINT8 *)pData;
    if(!MotorExchangeActive()) { MotorExchangeStop(); return; }
    EtherCATMotorBridge_SetLink(1,1);
    motor_outputs.control=MotorRead16(b); motor_outputs.speed=(INT16)MotorRead16(b+2);
    motor_outputs.sequence=MotorRead32(b+4);
    EtherCATMotorBridge_Receive(b,EMC_RX_BYTES);
}
void APPL_InputMapping(UINT16 *pData)
{
    if(!MotorExchangeActive()) MotorExchangeStop();
    EtherCATMotorBridge_Pack((UINT8 *)pData); MotorPublish();
}
void APPL_Application(void) { if(!MotorExchangeActive()) MotorExchangeStop(); MotorPublish(); }
UINT16 APPL_GetDeviceID(void) { return 0; }
