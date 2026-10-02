/* Full object access and SDO dispatcher: bounded descriptors and real writes. */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdoserv.c"
#include "objdef.c"

static UINT32 value = 0x12345678U;
static struct { UINT16 count; UINT32 entry; } recordValue = {1, 0x10203040U};
static const TSDOINFOENTRYDESC varDesc = {DEFTYPE_UNSIGNED32, 32, ACCESS_READ | ACCESS_WRITE};
static TOBJECT variable = {NULL, NULL, 0x2000, {DEFTYPE_UNSIGNED32, OBJCODE_VAR << 8}, &varDesc, NULL, &value, NULL, NULL, 0};
static TOBJECT record = {NULL, &variable, 0x1018, {DEFTYPE_RECORD, 1 | (OBJCODE_REC << 8)}, NULL, NULL, &recordValue, NULL, NULL, 0};
static TMBX response;
static unsigned sends, checks, customWrites;
UINT8 nAlStatus = STATE_PREOP;
UINT16 nPdInputSize = 6, nPdOutputSize = 2;
UINT16 u16SendMbxSize = 128, u16ReceiveMbxSize = 128;
TMBX *pCoeSendStored;

#define CHECK(expr) do { checks++; if (!(expr)) { \
    fprintf(stderr, "SSC objects failure line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)
const TOBJECT *COE_GetObjectDictionary(void) { return &record; }
UINT8 MBX_MailboxSendReq(TMBX *data, UINT8 flags) { sends++; memcpy(&response, data, sizeof(response)); return 0; }
UINT8 CustomWrite(UINT16 index, UINT8 subindex, UINT32 size, UINT16 *data, UINT8 complete)
{ customWrites++; memcpy(&value, data, 4); return 0; }

static void Request(UINT16 index, UINT8 subindex, UINT8 command, UINT32 size)
{
    TMBX request;
    TINITSDOMBX *sdo = (TINITSDOMBX *)&request;
    memset(&request, 0xa5, sizeof(request));
    sends = 0;
    request.MbxHeader.Length = EXPEDITED_FRAME_SIZE;
    sdo->SdoHeader.Sdo[0] = (UINT16)(command | ((index & 0xff) << 8));
    sdo->SdoHeader.Sdo[1] = (UINT16)((index >> 8) | (subindex << 8));
    if ((command & SDOHEADER_COMMAND) == SDOSERVICE_INITIATEDOWNLOADREQ && !(command & SDOHEADER_TRANSFERTYPE)) {
        request.MbxHeader.Length = (UINT16)(EXPEDITED_FRAME_SIZE + size);
        ((TINITSDODOWNLOADNORMREQMBX *)&request)->CompleteSize[0] = (UINT16)size;
        ((TINITSDODOWNLOADNORMREQMBX *)&request)->CompleteSize[1] = (UINT16)(size >> 16);
    }
    CHECK(SDOS_SdoInd(sdo) == 0);
    CHECK(sends == 1);
}
static void Abort(UINT32 code)
{
    CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[0] == SDOSERVICE_ABORTTRANSFER);
    CHECK(((TABORTSDOTRANSFERREQMBX *)&response)->AbortCode == code);
}
int main(int argc, char **argv)
{
    SYSTEM_INFO info;
    BYTE *pages;
    TSDOINFOENTRYDESC *descriptors;
    DWORD previous;
    unsigned handler, normal, size;
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    GetSystemInfo(&info);
    pages = VirtualAlloc(NULL, 2 * info.dwPageSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    CHECK(pages != NULL);
    CHECK(VirtualProtect(pages + info.dwPageSize, info.dwPageSize, PAGE_NOACCESS, &previous));
    descriptors = (TSDOINFOENTRYDESC *)(pages + info.dwPageSize - 2 * sizeof(TSDOINFOENTRYDESC));
    descriptors[0].DataType = DEFTYPE_UNSIGNED8; descriptors[0].BitLength = 8; descriptors[0].ObjAccess = ACCESS_READ;
    descriptors[1] = varDesc;
    record.pEntryDesc = descriptors;
    /* With the old dispatcher this touches the protected descriptor page. */
    if (argc == 1 || strcmp(argv[1], "widths") != 0) {
        Request(0x1018, 255, SDOSERVICE_INITIATEUPLOADREQ, 0); Abort(ABORT_SUBINDEX_NOT_EXISTING);
        Request(0x1018, 2, SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_TRANSFERTYPE, 4); Abort(ABORT_SUBINDEX_NOT_EXISTING);
        Request(0x2000, 1, SDOSERVICE_INITIATEUPLOADREQ, 0); Abort(ABORT_SUBINDEX_NOT_EXISTING);
        Request(0x1018, 1, SDOSERVICE_INITIATEUPLOADREQ, 0);
        CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[0] != SDOSERVICE_ABORTTRANSFER);
    }
    /* All rejected packets must leave both generic/custom object writes alone. */
    for (handler = 0; handler < 2; handler++) {
        variable.Write = handler ? CustomWrite : NULL;
        {
            UINT16 payload[3] = {0xa5a5, 0xa5a5, 0xa5a5};
            value = 0x12345678U; customWrites = 0;
            CHECK(OBJ_Write(0x2000, 0, 1, &variable, payload, 0) == ABORTIDX_PARAM_LENGTH_TOO_SHORT);
            CHECK(OBJ_Write(0x2000, 0, 6, &variable, payload, 0) == ABORTIDX_PARAM_LENGTH_TOO_LONG);
            CHECK(value == 0x12345678U && customWrites == 0);
            CHECK(OBJ_Write(0x2000, 0, 4, &variable, payload, 0) == 0);
            CHECK(value == 0xa5a5a5a5U && customWrites == handler);
        }
        for (normal = 0; normal < 2; normal++) {
            for (size = 1; size <= (normal ? 6U : 4U); size++) {
                UINT8 command = SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_SIZEINDICATOR;
                value = 0x12345678U; customWrites = 0;
                if (!normal) command |= SDOHEADER_TRANSFERTYPE | ((4-size) << SDOHEADERSHIFT_DATASETSIZE);
                Request(0x2000, 0, command, size);
                if (size < 4) { Abort(ABORT_PARAM_LENGTH_TOO_SHORT); CHECK(value == 0x12345678U && customWrites == 0); }
                else if (size > 4) { Abort(ABORT_PARAM_LENGTH_TOO_LONG); CHECK(value == 0x12345678U && customWrites == 0); }
                else { CHECK(value == 0xa5a5a5a5U); CHECK(customWrites == handler); }
            }
        }
    }
    /* Odd-address ENUM descriptions preserve all high pointer bits. */
    {
        TSDOINFOENTRYDESC enumDesc[2] = {{DEFTYPE_UNSIGNED8, 8, ACCESS_READ}, {DEFTYPE_OCTETSTRING, 48, ACCESS_READ}};
        CHAR *strings[1] = {(CHAR *)(pages + 513)};
        TOBJECT enumeration = {NULL, NULL, 0x0800, {DEFTYPE_RECORD, 1 | (OBJCODE_REC << 8)}, enumDesc, NULL, strings, NULL, NULL, 0};
        UINT16 data[3] = {0, 0, 0};
        memcpy(strings[0], "abcdef", 6);
        CHECK(OBJ_Read(0x0800, 1, 6, &enumeration, data, 0) == 0);
        CHECK(memcmp(data, "abcdef", 6) == 0);
    }
    /* Long expedited write to a one-byte descriptor is also rejected. */
    {
        TSDOINFOENTRYDESC byteDesc = {DEFTYPE_UNSIGNED8, 8, ACCESS_READ | ACCESS_WRITE};
        variable.pEntryDesc = &byteDesc;
        for (handler = 0; handler < 2; handler++) {
            variable.Write = handler ? CustomWrite : NULL; value = 0x12345678U; customWrites = 0;
            Request(0x2000, 0, SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_TRANSFERTYPE | SDOHEADER_SIZEINDICATOR, 4);
            Abort(ABORT_PARAM_LENGTH_TOO_LONG); CHECK(value == 0x12345678U && customWrites == 0);
        }
        variable.pEntryDesc = &varDesc; variable.Write = NULL;
    }
    /* Clearing PDO mapping SI0 must still permit configured entry writes. */
    {
        TSDOINFOENTRYDESC mappingDesc[2] = {{DEFTYPE_UNSIGNED8, 8, ACCESS_READ | ACCESS_WRITE}, {DEFTYPE_UNSIGNED32, 32, ACCESS_READ | ACCESS_WRITE}};
        struct { UINT16 count; UINT32 entry; } mappingValue = {0, 0};
        TOBJECT mapping = {NULL, NULL, 0x1600, {DEFTYPE_UNSIGNED32, 1 | (OBJCODE_ARR << 8)}, mappingDesc, NULL, &mappingValue, NULL, NULL, 0};
        variable.pNext = &mapping;
        Request(0x1600, 1, SDOSERVICE_INITIATEDOWNLOADREQ | SDOHEADER_TRANSFERTYPE | SDOHEADER_SIZEINDICATOR, 4);
        CHECK(((TINITSDOMBX *)&response)->SdoHeader.Sdo[0] == SDOSERVICE_INITIATEDOWNLOADRES);
        CHECK(mappingValue.count == 0 && mappingValue.entry == 0xa5a5a5a5U);
        Request(0x1600, 2, SDOSERVICE_INITIATEUPLOADREQ, 0); Abort(ABORT_SUBINDEX_NOT_EXISTING);
        variable.pNext = NULL;
    }
    CHECK(VirtualFree(pages, 0, MEM_RELEASE));
    printf("SSC real object access: %u checks passed\n", checks);
    return 0;
}
