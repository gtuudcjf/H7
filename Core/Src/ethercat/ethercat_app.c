#include "ethercat_app.h"
#include "ethercat_app_core.h"
#include "ecatslv.h"
#include "el9800hw.h"

/* SI0 占 UINT16；其余成员按 SSC OBJ_DWORD_ALIGN 自然对齐。
   对象存储布局与线上 PDO 布局不同，不能直接 memcpy 到过程数据。 */
static struct { UINT16 count; UINT32 entries[1]; } rx_map = {1, {0x70000110U}};
static struct { UINT16 count; UINT32 entries[2]; } tx_map = {2, {0x60000110U, 0x60000220U}};
static struct { UINT16 count; UINT16 entry; } rx_assign = {1, 0x1600};
static struct { UINT16 count; UINT16 entry; } tx_assign = {1, 0x1a00};
static struct { UINT16 count; UINT16 status; UINT32 counter; } inputs = {2, 0, 0};
static struct { UINT16 count; UINT16 control; } outputs = {1, 0};
static BOOL valid_output;

static const TSDOINFOENTRYDESC map_desc[] = {
    {DEFTYPE_UNSIGNED8, 8, ACCESS_READ},
    {DEFTYPE_UNSIGNED32, 32, ACCESS_READ},
    {DEFTYPE_UNSIGNED32, 32, ACCESS_READ}
};
static const TSDOINFOENTRYDESC assign_desc[] = {
    {DEFTYPE_UNSIGNED8, 8, ACCESS_READ},
    {DEFTYPE_UNSIGNED16, 16, ACCESS_READ}
};
static const TSDOINFOENTRYDESC input_desc[] = {
    {DEFTYPE_UNSIGNED8, 8, ACCESS_READ},
    {DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
    {DEFTYPE_UNSIGNED32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING}
};
static const TSDOINFOENTRYDESC output_desc[] = {
    {DEFTYPE_UNSIGNED8, 8, ACCESS_READ},
    {DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING}
};
static const UCHAR rx_map_name[] = "RxPDO map\000\377";
static const UCHAR tx_map_name[] = "TxPDO map\000\377";
static const UCHAR rx_assign_name[] = "RxPDO assign\000\377";
static const UCHAR tx_assign_name[] = "TxPDO assign\000\377";
static const UCHAR input_name[] = "Inputs\000StatusWord\000CycleCounter\000\377";
static const UCHAR output_name[] = "Outputs\000ControlWord\000\377";

/* 0x1018 由 coeappl.c 和共享身份宏提供，应用不重复注册。 */
TOBJECT OBJMEM ApplicationObjDic[] = {
    {NULL, NULL, 0x1600, {DEFTYPE_PDOMAPPING, 1 | (OBJCODE_REC << 8)}, map_desc, rx_map_name, &rx_map, NULL, NULL, 0},
    {NULL, NULL, 0x1a00, {DEFTYPE_PDOMAPPING, 2 | (OBJCODE_REC << 8)}, map_desc, tx_map_name, &tx_map, NULL, NULL, 0},
    {NULL, NULL, 0x1c12, {DEFTYPE_UNSIGNED16, 1 | (OBJCODE_ARR << 8)}, assign_desc, rx_assign_name, &rx_assign, NULL, NULL, 0},
    {NULL, NULL, 0x1c13, {DEFTYPE_UNSIGNED16, 1 | (OBJCODE_ARR << 8)}, assign_desc, tx_assign_name, &tx_assign, NULL, NULL, 0},
    {NULL, NULL, 0x6000, {DEFTYPE_RECORD, 2 | (OBJCODE_REC << 8)}, input_desc, input_name, &inputs, NULL, NULL, 0},
    {NULL, NULL, 0x7000, {DEFTYPE_RECORD, 1 | (OBJCODE_REC << 8)}, output_desc, output_name, &outputs, NULL, NULL, 0},
    {NULL, NULL, 0xffff, {0, 0}, NULL, NULL, NULL, NULL, NULL, 0}
};

static void ResetProcessData(void)
{
    valid_output = FALSE;
    EtherCAT_AppReset();
    outputs.control = 0;
    inputs.status = 0;
    inputs.counter = 0;
}

static BOOL ExchangeActive(void)
{
    return bEcatInputUpdateRunning && bEcatOutputUpdateRunning &&
           bEcatFirstOutputsReceived && !HW_AccessFault();
}

void APPL_AckErrorInd(UINT16 stateTrans)
{
    (void)stateTrans;
    ResetProcessData();
}

UINT16 APPL_StartMailboxHandler(void) { ResetProcessData(); return 0; }
UINT16 APPL_StopMailboxHandler(void) { ResetProcessData(); return 0; }
UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    (void)pIntMask; /* FreeRun 不增加中断掩码。 */
    ResetProcessData();
    return 0;
}
UINT16 APPL_StopInputHandler(void) { ResetProcessData(); return 0; }
UINT16 APPL_StartOutputHandler(void) { ResetProcessData(); return 0; }
UINT16 APPL_StopOutputHandler(void) { ResetProcessData(); return 0; }

UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize)
{
    *pInputSize = 6;
    *pOutputSize = 2;
    return 0;
}

void APPL_OutputMapping(UINT16 *pData)
{
    if (!ExchangeActive()) {
        ResetProcessData();
        return;
    }
    EtherCAT_AppConsumeRx((const uint8_t *)pData);
    outputs.control = EtherCAT_AppGetRx()->control_word;
    valid_output = TRUE;
}

void APPL_InputMapping(UINT16 *pData)
{
    if (!ExchangeActive() || !valid_output) ResetProcessData();
    EtherCAT_AppProduceTx((uint8_t *)pData);
}

void APPL_Application(void)
{
    uint8_t rx[2];
    if (!ExchangeActive() || !valid_output) {
        ResetProcessData();
        return;
    }
    /* SDO 写入 ControlWord 与 PDO 共用影子，但不能代替有效 PDO 接收。
       计数单位是有效 FreeRun 应用迭代，不保证等于主站周期数。 */
    rx[0] = (uint8_t)outputs.control;
    rx[1] = (uint8_t)(outputs.control >> 8);
    EtherCAT_AppConsumeRx(rx);
    EtherCAT_AppCycle();
    inputs.status = EtherCAT_AppGetTx()->status_word;
    inputs.counter = EtherCAT_AppGetTx()->cycle_counter;
}

UINT16 APPL_GetDeviceID(void)
{
    return 0; /* EXPLICIT_DEVICE_ID=0，没有拨码开关身份。 */
}
