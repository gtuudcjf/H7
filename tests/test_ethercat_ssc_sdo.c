/* Real generated SSC SDO dispatcher, with fake dictionary/mailbox boundaries. */
#include <stdio.h>
#include <stdlib.h>
#include "sdoserv.c"
#include "ecatcoe.c"

static const TOBJECT object = {0};
static UINT32 objectLength = 4;
static UINT8 accessResult;
static unsigned reads, writes, sends, checks, allocations;
static TMBX response;
static TMBX continuationBuffer;
UINT16 u16SendMbxSize = 128, u16ReceiveMbxSize = 128;
CHAR aSubindexDesc[13];
TSYNCMANPAR sSyncManOutPar, sSyncManInPar;
BOOL bSyncSetByUser;
UINT16 nPdInputSize = 6, nPdOutputSize = 2;
UINT16 COE_ObjDictionaryInit(void) { return 0; }
void COE_ClearObjDictionary(void) { }
#include "ssc_freerun_bodies.h"

#define CHECK(expr) do { checks++; if (!(expr)) { \
    fprintf(stderr, "SSC SDO failure line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)

const TOBJECT *OBJ_GetObjectHandle(UINT16 index) { return &object; }
UINT32 OBJ_GetObjectLength(UINT16 index, UINT8 subindex, const TOBJECT *entry, UINT8 complete) { return objectLength; }
UINT8 OBJ_Read(UINT16 index, UINT8 subindex, UINT32 size, const TOBJECT *entry, UINT16 *data, UINT8 complete)
{ reads++; if (data != NULL && size <= 112) memset(data, 0x12, size); return accessResult; }
UINT8 OBJ_Write(UINT16 index, UINT8 subindex, UINT32 size, const TOBJECT *entry, UINT16 *data, UINT8 complete)
{ writes++; return accessResult; }
UINT8 MBX_MailboxSendReq(TMBX *data, UINT8 flags) { sends++; memcpy(&response, data, sizeof(response)); return 0; }
UINT16 OBJ_GetNoOfObjects(UINT8 type) { return 100; }
UINT16 OBJ_GetObjectList(UINT16 type, UINT16 *index, UINT16 size, UINT16 *data, UINT8 *abort) { return 0; }
UINT16 OBJ_GetDesc(UINT16 index, UINT8 subindex, const TOBJECT *entry, UINT16 *data) { return 0; }
const TSDOINFOENTRYDESC *OBJ_GetEntryDesc(const TOBJECT *entry, UINT8 subindex) { return NULL; }
const TSDOINFOOBJDESC *OBJ_GetObjDesc(const TOBJECT *entry) { return NULL; }
void *HW_AllocMailboxBuffer(UINT16 size)
{ CHECK(size == 128); allocations++; return &continuationBuffer; }
void HW_FreeMailboxBuffer(void *pointer) { }

static void Request(UINT8 command, UINT32 length, UINT8 result)
{
    TMBX request;
    memset(&request, 0, sizeof(request));
    memset(&response, 0, sizeof(response));
    objectLength = length; accessResult = result; reads = writes = sends = 0;
    request.MbxHeader.Length = EXPEDITED_FRAME_SIZE;
    ((TINITSDOMBX *)&request)->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] = command;
    if ((command & SDOHEADER_COMMAND) == SDOSERVICE_INITIATEDOWNLOADREQ && !(command & SDOHEADER_TRANSFERTYPE))
    {
        request.MbxHeader.Length = length > 112 ? 122 : (UINT16)(10 + length);
        ((TINITSDODOWNLOADNORMREQMBX *)&request)->CompleteSize[0] = (UINT16)length;
    }
    CHECK(SDOS_SdoInd((TINITSDOMBX *)&request) == 0);
}

static void ExpectAbort(UINT32 code)
{
    CHECK(sends == 1);
    CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] == SDOSERVICE_ABORTTRANSFER);
    CHECK(((TABORTSDOTRANSFERREQMBX *)&response)->AbortCode == code);
    CHECK(!bSdoInWork && !bSdoSegFollows && pSdoSegData == NULL);
}

int main(void)
{
    COE_ObjInit();
    CHECK((sSyncManOutPar.u16SyncTypesSupported & (SYNCTYPE_SYNCHRONSUPP | SYNCTYPE_DCSYNC0SUPP | SYNCTYPE_DCSYNC1SUPP | SYNCTYPE_SUBCYCLESUPP)) == 0);
    CHECK(sSyncManInPar.u16SyncTypesSupported == sSyncManOutPar.u16SyncTypesSupported);
    CHECK(SelectSync(0) == 0);
    CHECK(sSyncManOutPar.u16SyncType == SYNCTYPE_FREERUN && sSyncManInPar.u16SyncType == SYNCTYPE_FREERUN);
    CHECK(SelectSync(ESC_DC_SYNC_UNIT_ACTIVE_MASK | ESC_DC_SYNC0_ACTIVE_MASK) == ALSTATUSCODE_DCINVALIDSYNCCFG);
    CHECK(sSyncManOutPar.u16SyncType == SYNCTYPE_FREERUN && sSyncManInPar.u16SyncType == SYNCTYPE_FREERUN);
    CHECK(SelectSync(ESC_DC_SYNC_UNIT_AUTO_ACTIVE_MASK) == ALSTATUSCODE_DCINVALIDSYNCCFG);
    nPdOutputSize = 0;
    CHECK(SelectSync(0) == 0 && sSyncManInPar.u16SyncType == SYNCTYPE_FREERUN);
    nPdOutputSize = 2; nPdInputSize = 0;
    CHECK(SelectSync(0) == 0 && sSyncManOutPar.u16SyncType == SYNCTYPE_FREERUN);
    nPdInputSize = 6;
    bSyncSetByUser = TRUE; sSyncManInPar.u16SyncType = SYNCTYPE_SM_SYNCHRON;
    CHECK(SelectSync(0) == ALSTATUSCODE_SYNCHRONNOTSUPPORTED);
    COE_ObjInit();
    Request(SDOSERVICE_INITIATEUPLOADREQ | SDOHEADER_COMPLETEACCESS, 4, 0);
    ExpectAbort(ABORT_COMPLETE_ACCESS_NOT_SUPPORTED);
    CHECK(reads == 0 && writes == 0);
    Request(SDOSERVICE_INITIATEUPLOADREQ, 1024, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    CHECK(reads == 0 && writes == 0);
    Request(SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_SIZEINDICATOR, 1024, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    CHECK(reads == 0 && writes == 0);
    Request(SDOSERVICE_DOWNLOADSEGMENTREQ, 4, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    Request(SDOSERVICE_UPLOADSEGMENTREQ, 4, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    Request(SDOSERVICE_DOWNLOADSEGMENTREQ | SEGHEADER_TOGGLE, 4, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    Request(SDOSERVICE_UPLOADSEGMENTREQ | SEGHEADER_TOGGLE, 4, 0);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    Request(SDOSERVICE_INITIATEUPLOADREQ, 4, ABORTIDX_WORKING);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    CHECK(reads == 1 && writes == 0);
    Request(SDOSERVICE_INITIATEUPLOADREQ, 4, 0);
    CHECK(sends == 1 && reads == 1 && writes == 0);
    CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] != SDOSERVICE_ABORTTRANSFER);
    Request(SDOSERVICE_INITIATEUPLOADREQ, 6, 0);
    CHECK(sends == 1 && reads == 1 && writes == 0);
    CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] != SDOSERVICE_ABORTTRANSFER);
    Request(SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_TRANSFERTYPE | SDOHEADER_SIZEINDICATOR, 4, 0);
    CHECK(sends == 1 && writes == 1 && reads == 0);
    CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] == SDOSERVICE_INITIATEDOWNLOADRES);
    Request(SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_TRANSFERTYPE | SDOHEADER_SIZEINDICATOR, 4, ABORTIDX_WORKING);
    ExpectAbort(ABORT_UNSUPPORTED_ACCESS);
    CHECK(writes == 1);
    {
        TMBX request;
        TSDOINFORMATION *info = (TSDOINFORMATION *)&request;
        memset(&request, 0, sizeof(request));
        info->MbxHeader.Length = SIZEOF_SDOINFOLISTSTRUCT;
        info->SdoHeader.InfoHead = SDOINFOSERVICE_OBJDICTIONARYLIST_Q << INFOHEAD_OPCODE_SHIFT;
        info->SdoHeader.Data.List.ListType = 1;
        sends = allocations = 0;
        CHECK(SDOS_SdoInfoInd(info) == 0);
        CHECK(sends == 1 && nSdoInfoFragmentsLeft == 1);
        CHECK(COE_ContinueInd(NULL) == 0);
        CHECK(allocations == 1 && sends == 2 && nSdoInfoFragmentsLeft == 0);
        CHECK(((TSDOINFORMATION *)&response)->SdoHeader.FragmentsLeft == 0);
    }
    printf("SSC SDO configuration: %u checks passed\n", checks);
    return 0;
}
