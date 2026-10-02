/* Explicit wire bytes and the real SSC dictionary/lifecycle contract. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "ethercat_app_core.h"

static unsigned checks;
#define CHECK(x) do { checks++; if (!(x)) { fprintf(stderr, "EtherCAT app failure line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static void CoreTests(void)
{
    uint8_t rx[2] = {0x35, 0x92};
    uint8_t bytes[8];
    const uint8_t expected[6] = {1, 0, 1, 0, 0, 0};
    EtherCAT_AppReset();
    CHECK(sizeof(EtherCATRxPdo) == 2);
    CHECK(EtherCAT_AppGetRx()->control_word == 0);
    CHECK(EtherCAT_AppGetTx()->cycle_counter == 0);
    EtherCAT_AppConsumeRx(rx);
    CHECK(EtherCAT_AppGetRx()->control_word == 0x9235);
    EtherCAT_AppCycle();
    memset(bytes, 0xa5, sizeof(bytes));
    EtherCAT_AppProduceTx(bytes + 1);
    CHECK(memcmp(bytes + 1, expected, 6) == 0);
    CHECK(bytes[0] == 0xa5 && bytes[7] == 0xa5);
    rx[0] = 0xfe; rx[1] = 0xff;
    EtherCAT_AppConsumeRx(rx); EtherCAT_AppCycle();
    CHECK(EtherCAT_AppGetTx()->status_word == 0);
    CHECK(EtherCAT_AppGetTx()->cycle_counter == 2);
    /* Seed the existing mutable storage through its const view only in tests;
       no production API exists solely to accelerate the rollover test. */
    ((EtherCATTxPdo *)EtherCAT_AppGetTx())->cycle_counter = UINT32_MAX;
    EtherCAT_AppCycle();
    CHECK(EtherCAT_AppGetTx()->cycle_counter == 0);
    ((EtherCATTxPdo *)EtherCAT_AppGetTx())->cycle_counter = UINT32_C(0x78563412);
    EtherCAT_AppProduceTx(bytes + 1);
    CHECK(bytes[3] == 0x12 && bytes[4] == 0x34 && bytes[5] == 0x56 && bytes[6] == 0x78);
    EtherCAT_AppReset();
    CHECK(EtherCAT_AppGetRx()->control_word == 0);
    CHECK(EtherCAT_AppGetTx()->status_word == 0 && EtherCAT_AppGetTx()->cycle_counter == 0);
}

#ifdef ETHERCAT_APP_SSC_TEST
#include "sdoserv.c"
#include "objdef.c"
#include "coeappl.c"
#include "ethercat_app.h"

UINT8 nAlStatus = STATE_PREOP;
UINT16 nPdInputSize = 6, nPdOutputSize = 2;
UINT16 u16SendMbxSize = 128, u16ReceiveMbxSize = 128;
BOOL bEcatInputUpdateRunning, bEcatOutputUpdateRunning, bEcatFirstOutputsReceived;
TMBX *pCoeSendStored;
static BOOL fault;
static TMBX response;
static unsigned sends;
BOOL HW_AccessFault(void) { return fault; }
UINT8 MBX_MailboxSendReq(TMBX *data, UINT8 flags)
{ sends++; memcpy(&response, data, sizeof(response)); return 0; }

static UINT32 Read(UINT16 index, UINT8 subindex, UINT32 size)
{
    UINT16 data[4] = {0, 0, 0, 0};
    UINT32 value = 0;
    const TOBJECT *object = OBJ_GetObjectHandle(index);
    CHECK(object != NULL);
    CHECK(OBJ_GetObjectLength(index, subindex, object, 0) == size);
    CHECK(OBJ_Read(index, subindex, size, object, data, 0) == 0);
    memcpy(&value, data, size);
    return value;
}
static void Upload(UINT16 index, UINT8 subindex, UINT32 size, UINT32 expected)
{
    TMBX request;
    TINITSDOMBX *sdo = (TINITSDOMBX *)&request;
    UINT32 value = 0;
    memset(&request, 0, sizeof(request)); sends = 0;
    request.MbxHeader.Length = EXPEDITED_FRAME_SIZE;
    sdo->SdoHeader.Sdo[0] = (UINT16)(SDOSERVICE_INITIATEUPLOADREQ | ((index & 255) << 8));
    sdo->SdoHeader.Sdo[1] = (UINT16)((index >> 8) | (subindex << 8));
    CHECK(SDOS_SdoInd(sdo) == 0 && sends == 1);
    CHECK((((TINITSDOMBX *)&response)->SdoHeader.Sdo[0] & SDOHEADER_COMMAND) == SDOSERVICE_INITIATEUPLOADRES);
    CHECK((((TINITSDOMBX *)&response)->SdoHeader.Sdo[0] & SDOHEADER_DATASETSIZE) == ((4 - size) << SDOHEADERSHIFT_DATASETSIZE));
    memcpy(&value, ((UINT8 *)&response) + MBX_HEADER_SIZE + COE_HEADER_SIZE + 4, size);
    CHECK(value == expected);
}
static void Safe(void)
{
    CHECK(Read(0x7000, 1, 2) == 0);
    CHECK(Read(0x6000, 1, 2) == 0);
    CHECK(Read(0x6000, 2, 4) == 0);
    CHECK(EtherCAT_AppGetRx()->control_word == 0);
}
static void Activate(void)
{
    UINT16 mask = 0xa55a;
    CHECK(APPL_StartInputHandler(&mask) == 0 && mask == 0xa55a);
    bEcatInputUpdateRunning = TRUE;
    CHECK(APPL_StartOutputHandler() == 0);
    bEcatOutputUpdateRunning = TRUE;
    bEcatFirstOutputsReceived = TRUE;
    nAlStatus = STATE_OP;
}
static void Exchange(void)
{
    UINT8 rx[4] = {0xa5, 0x31, 0xa5, 0xa5};
    UINT8 tx[8];
    const UINT8 expected[6] = {1, 0, 1, 0, 0, 0};
    memset(tx, 0xa5, sizeof(tx));
    APPL_OutputMapping((UINT16 *)(rx + 1)); APPL_Application(); APPL_InputMapping((UINT16 *)(tx + 1));
    CHECK(memcmp(tx + 1, expected, 6) == 0 && tx[0] == 0xa5 && tx[7] == 0xa5);
    CHECK(Read(0x7000, 1, 2) == 0xa531);
    CHECK(Read(0x6000, 1, 2) == 1);
    CHECK(Read(0x6000, 2, 4) == 1);
}
static void SscTests(void)
{
    UINT16 input = 0, output = 0, data = 0x1234;
    UINT16 indices[] = {0x1600, 0x1a00, 0x1c12, 0x1c13, 0x6000};
    unsigned i, identities = 0;
    const TOBJECT *object;
    CHECK(COE_ObjDictionaryInit() == 0);
    for (object = COE_GetObjectDictionary(); object; object = object->pNext) if (object->Index == 0x1018) identities++;
    CHECK(identities == 1);
    CHECK(APPL_GenerateMapping(&input, &output) == 0 && input == 6 && output == 2);
    CHECK(Read(0x1018, 0, 1) == 4);
    CHECK(Read(0x1018, 1, 4) == 0x00f0c743);
    CHECK(Read(0x1018, 2, 4) == 0x9252);
    CHECK(Read(0x1018, 3, 4) == 1 && Read(0x1018, 4, 4) == 1);
    CHECK(Read(0x1600, 0, 1) == 1 && Read(0x1600, 1, 4) == 0x70000110);
    CHECK(Read(0x1a00, 0, 1) == 2);
    CHECK(Read(0x1a00, 1, 4) == 0x60000110 && Read(0x1a00, 2, 4) == 0x60000220);
    CHECK(Read(0x1c12, 0, 1) == 1 && Read(0x1c12, 1, 2) == 0x1600);
    CHECK(Read(0x1c13, 0, 1) == 1 && Read(0x1c13, 1, 2) == 0x1a00);
    CHECK(OBJ_GetEntryOffset(1, OBJ_GetObjectHandle(0x1600)) == 32);
    CHECK(OBJ_GetEntryOffset(2, OBJ_GetObjectHandle(0x1a00)) == 64);
    CHECK(OBJ_GetEntryOffset(1, OBJ_GetObjectHandle(0x6000)) == 16);
    CHECK(OBJ_GetEntryOffset(2, OBJ_GetObjectHandle(0x6000)) == 32);
    Upload(0x1018, 1, 4, 0x00f0c743); Upload(0x1600, 1, 4, 0x70000110);
    Upload(0x1a00, 2, 4, 0x60000220); Upload(0x1c12, 1, 2, 0x1600);
    for (i = 0; i < sizeof(indices)/sizeof(indices[0]); i++) {
        object = OBJ_GetObjectHandle(indices[i]);
        CHECK(OBJ_Write(indices[i], 0, 1, object, &data, 0) == ABORTIDX_READ_ONLY_ENTRY);
        CHECK(OBJ_Write(indices[i], 1, (i < 2) ? 4 : 2, object, &data, 0) == ABORTIDX_READ_ONLY_ENTRY);
    }
    CHECK(OBJ_Write(0x6000, 2, 4, OBJ_GetObjectHandle(0x6000), &data, 0) == ABORTIDX_READ_ONLY_ENTRY);
    CHECK(OBJ_Write(0x7000, 0, 1, OBJ_GetObjectHandle(0x7000), &data, 0) == ABORTIDX_READ_ONLY_ENTRY);
    CHECK(OBJ_Write(0x7000, 1, 1, OBJ_GetObjectHandle(0x7000), &data, 0) == ABORTIDX_PARAM_LENGTH_TOO_SHORT);
    CHECK(OBJ_Write(0x7000, 1, 4, OBJ_GetObjectHandle(0x7000), &data, 0) == ABORTIDX_PARAM_LENGTH_TOO_LONG);
    CHECK(APPL_StartMailboxHandler() == 0); APPL_Application(); Safe();
    CHECK(OBJ_Write(0x7000, 1, 2, OBJ_GetObjectHandle(0x7000), &data, 0) == 0);
    CHECK(Read(0x7000, 1, 2) == 0x1234);
    APPL_Application(); Safe(); /* PREOP cannot count or retain unsafe control. */
    Activate(); APPL_Application(); Safe(); /* OP before first valid mapping. */
    Exchange(); APPL_Application(); CHECK(Read(0x6000, 2, 4) == 2);
    Upload(0x6000, 1, 2, 1); Upload(0x6000, 2, 4, 2);
    data = 0xfffe;
    CHECK(OBJ_Write(0x7000, 1, 2, OBJ_GetObjectHandle(0x7000), &data, 0) == 0);
    APPL_Application(); CHECK(Read(0x6000, 1, 2) == 0);
    CHECK(Read(0x7000, 1, 2) == 0xfffe);
    CHECK(APPL_StopOutputHandler() == 0); bEcatOutputUpdateRunning = FALSE;
    APPL_Application(); Safe();
    Activate(); APPL_Application(); Safe(); Exchange();
    APPL_AckErrorInd(0x84); APPL_Application(); Safe();
    Exchange(); bEcatFirstOutputsReceived = FALSE; APPL_Application(); Safe();
    bEcatFirstOutputsReceived = TRUE; APPL_Application(); Safe();
    Exchange(); fault = TRUE;
    {
        UINT8 tx[8] = {0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5};
        const UINT8 zero[6] = {0, 0, 0, 0, 0, 0};
        APPL_InputMapping((UINT16 *)(tx + 1));
        CHECK(memcmp(tx + 1, zero, 6) == 0 && tx[0] == 0xa5 && tx[7] == 0xa5);
    }
    APPL_Application(); Safe();
    data = 1; APPL_OutputMapping(&data); fault = FALSE; APPL_Application(); Safe();
    Exchange(); CHECK(APPL_StopInputHandler() == 0); bEcatInputUpdateRunning = FALSE;
    APPL_Application(); Safe();
    Activate(); Exchange(); CHECK(APPL_StopMailboxHandler() == 0); APPL_Application(); Safe();
    CHECK(APPL_GetDeviceID() == 0); /* Explicit ID is disabled; no board switches. */
    COE_ClearObjDictionary();
}
#endif

int main(void)
{
    CoreTests();
#ifdef ETHERCAT_APP_SSC_TEST
    SscTests();
#endif
    printf("EtherCAT app: %u checks passed\n", checks);
    return 0;
}
