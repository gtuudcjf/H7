/* Real mailbox -> CoE -> SDO/SDOInfo -> dictionary -> ESC send chain.
 * Only the physical ESC and fixed allocator boundaries are simulated. */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "mailbox.c"
#include "ecatcoe.c"
#include "sdoserv.c"
#include "objdef.c"
#include "coeappl.c"

static TMBX *pool[24];
static BYTE *pages[24];
static unsigned owned[24], checks, writes, receiveCopy;
static TSYNCMAN syncManagers[2];
static unsigned char receive[128], sent[128];
UINT8 nAlStatus = STATE_PREOP;
volatile UINT16 u16dummy;
UINT16 nPdInputSize = 6, nPdOutputSize = 2;
BOOL bEcatInputUpdateRunning, bEcatOutputUpdateRunning, bEcatFirstOutputsReceived;

#define CHECK(x) do { checks++; if (!(x)) { \
    fprintf(stderr, "SSC lengths failure line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
BOOL HW_AccessFault(void) { return FALSE; }
void *HW_AllocMailboxBuffer(UINT16 size)
{
    unsigned i;
    CHECK(size <= 128);
    for (i = 0; i < 24; i++) if (!owned[i]) {
        owned[i] = 1; memset(pool[i], 0xa5, 128); return pool[i];
    }
    return NULL;
}
void HW_FreeMailboxBuffer(void *pointer)
{
    unsigned i;
    if (!pointer || pointer == &asMbx[0] || pointer == &asMbx[1]) return;
    for (i = 0; i < 24; i++) if (pointer == pool[i]) {
        CHECK(owned[i] == 1); owned[i] = 0; return;
    }
    CHECK(0);
}
void HW_EscRead(MEM_ADDR *data, UINT16 address, UINT16 length)
{
    if (address >= 0x1000 && address < 0x1080) {
        CHECK(address + length <= 0x1000 + u16ReceiveMbxSize);
        CHECK(address + length <= 0x1080);
        if (address == 0x1000 && length != 2) receiveCopy = length;
        memcpy(data, receive + address - 0x1000, length);
    } else memset(data, 0, length); /* SM reports empty: exercise last-word trigger. */
}
void HW_EscWrite(MEM_ADDR *data, UINT16 address, UINT16 length)
{
    unsigned i;
    if (address < 0x1080 || address >= 0x1100) return;
    CHECK(address + length <= 0x1080 + u16SendMbxSize);
    CHECK(address + length <= 0x1100);
    for (i = 0; i < 24; i++) if ((BYTE *)data >= (BYTE *)pool[i] &&
        (BYTE *)data < (BYTE *)pool[i] + 128) {
        CHECK((BYTE *)data + length <= (BYTE *)pool[i] + 128);
        CHECK(owned[i]);
    }
    if (address == 0x1080) writes++;
    memcpy(sent + address - 0x1080, data, length);
}
void DisableSyncManChannel(UINT8 channel) { }
void EnableSyncManChannel(UINT8 channel) { }
TSYNCMAN *GetSyncMan(UINT8 channel) { return &syncManagers[channel]; }

static void Start(UINT16 size)
{
    MBX_Init(); COE_Init();
    syncManagers[0].PhysicalStartAddress = 0x1000; syncManagers[0].Length = size;
    syncManagers[1].PhysicalStartAddress = 0x1080; syncManagers[1].Length = size;
    CHECK(MBX_StartMailboxHandler() == 0);
    bMbxRunning = TRUE; /* AL transition enables processing after the start hook. */
    writes = receiveCopy = 0; memset(sent, 0xa5, 128);
}
static void Recover(void)
{
    unsigned i;
    TMBX *all[24];
    MBX_StopMailboxHandler();
    CHECK(!pCoeSendStored && !nSdoInfoFragmentsLeft && !u8MailboxSendReqStored);
    for (i = 0; i < 24; i++) { CHECK(!owned[i]); all[i] = HW_AllocMailboxBuffer(128); CHECK(all[i]); }
    CHECK(HW_AllocMailboxBuffer(128) == NULL);
    for (i = 0; i < 24; i++) HW_FreeMailboxBuffer(all[i]);
}
static void ValidAfterError(void)
{
    const unsigned char upload[] = {10,0,0,0,0,3,0,0x20,0x40,0x18,0x10,1,0,0,0,0};
    const unsigned char identity[] = {0x43,0xc7,0xf0,0};
    MBX_MailboxReadInd(); writes = 0; memset(sent, 0xa5, 128);
    memset(receive, 0xa5, 128); memcpy(receive, upload, 16);
    MBX_CheckAndCopyMailbox(); MBX_Main();
    CHECK(writes == 1 && sent[0] == 10 && sent[8] == 0x43);
    CHECK(memcmp(sent + 12, identity, 4) == 0);
}
static void Request(UINT16 length, UINT16 service, UINT8 opcode, UINT16 index, UINT8 subindex)
{
    memset(receive, 0xa5, 128);
    receive[0] = (UINT8)length; receive[1] = (UINT8)(length >> 8);
    receive[2] = receive[3] = receive[4] = 0; receive[5] = 3;
    receive[6] = 0; receive[7] = (UINT8)(service << 4);
    receive[8] = opcode; receive[9] = receive[10] = receive[11] = 0;
    receive[12] = (UINT8)index; receive[13] = (UINT8)(index >> 8);
    receive[14] = subindex; receive[15] = 0;
    MBX_CheckAndCopyMailbox(); MBX_Main();
    CHECK(writes == 1);
}
static void MbxError(UINT8 code)
{
    const unsigned char prefix[5] = {4,0,0,0,0};
    CHECK(memcmp(sent, prefix, 5) == 0);
    CHECK((sent[5] & 15) == 0);
    CHECK(sent[6] == 1 && sent[7] == 0 && sent[8] == code && sent[9] == 0);
    CHECK(sent[10] == 0xa5);
}
static void Ingress(void)
{
    static const UINT16 lengths[] = {123,128,129,0xffff};
    unsigned i;
    for (i = 0; i < 4; i++) {
        Start(128); Request(lengths[i], 8, 1, 6, 0);
        CHECK(receiveCopy == 6); MbxError(8); ValidAfterError(); Recover();
    }
    Start(34); Request(29, 8, 1, 6, 0); CHECK(receiveCopy == 6); MbxError(8); Recover();
}
static void NameWire(UINT16 size)
{
    Start(size); Request(10, 8, 5, 0x1c32, 4);
    CHECK(sent[0] == (size == 34 ? 16 : 47) && sent[1] == 0 && sent[8] == 6);
    if (size == 34) CHECK(sent[22] == 0xa5);
    else {
        CHECK(memcmp(sent + 22, "Synchronization Types supported", 31) == 0);
        CHECK(sent[53] == 0xa5);
    }
    Recover();
}
static void Names(void)
{
    UINT16 name[24];
    const TOBJECT *object = OBJ_GetObjectHandle(0x1c32);
    CHECK(object);
    CHECK(OBJ_GetDesc(0x1c32, 4, object, NULL) == 31);
    memset(name, 0xa5, sizeof(name));
    CHECK(OBJ_GetDesc(0x1c32, 4, object, name) == 31);
    CHECK(memcmp(name, "Synchronization Types supported", 31) == 0);
    CHECK(((BYTE *)name)[31] == 0xa5);
    NameWire(34); NameWire(128);
}
static void ShortHeaders(void)
{
    static const UINT16 lengths[] = {0,1,2,3,5,6,7,9};
    unsigned i;
    for (i = 0; i < 8; i++) {
        Start(128); Request(lengths[i], 8, 5, 0x1c32, 4); MbxError(6); Recover();
    }
    for (i = 0; i < 10; i++) {
        Start(128); Request((UINT16)i, 2, 0x50, 0, 0); MbxError(6); Recover();
    }
}
static void InvalidList(void)
{
    static const UINT16 lengths[] = {8,122};
    unsigned i;
    for (i = 0; i < 2; i++) {
        Start(128); Request(lengths[i], 8, 1, 6, 0);
        CHECK(receiveCopy == (i ? 128 : 14));
        CHECK(sent[0] == 10 && sent[1] == 0 && (sent[5] & 15) == 3);
        CHECK(sent[8] == 7 && sent[10] == 0 && sent[11] == 0);
        CHECK(sent[12] == 1 && sent[13] == 0 && sent[14] == 4 && sent[15] == 5);
        CHECK(sent[16] == 0xa5); ValidAfterError(); Recover();
    }
}
static void SendBounds(void)
{
    static const UINT16 lengths[] = {123,128,129,0xffff};
    unsigned i, busy;
    for (busy = 0; busy < 2; busy++) for (i = 0; i < 4; i++) {
        TMBX *buffer;
        Start(128); bSendMbxIsFull = (BOOL)busy;
        buffer = HW_AllocMailboxBuffer(128); memset(&buffer->MbxHeader, 0, 6);
        buffer->MbxHeader.Length = lengths[i];
        nSdoInfoFragmentsLeft = 3;
        u8MailboxSendReqStored = COE_SERVICE;
        CHECK(MBX_MailboxSendReq(buffer, COE_SERVICE | FRAGMENTS_FOLLOW) == 0);
        CHECK(!nSdoInfoFragmentsLeft && !u8MailboxSendReqStored);
        if (busy) { CHECK(writes == 0); MBX_MailboxReadInd(); }
        CHECK(writes == 1); MbxError(8); Recover();
    }
}
static void MaximumSend(void)
{
    TMBX *buffer;
    Start(128); buffer = HW_AllocMailboxBuffer(128); buffer->MbxHeader.Length = 122;
    ((BYTE *)buffer)[126] = 0x31; ((BYTE *)buffer)[127] = 0x72;
    CHECK(MBX_MailboxSendReq(buffer, 0) == 0); CHECK(writes == 1);
    CHECK(sent[126] == 0x31 && sent[127] == 0x72); Recover();
    Start(35); buffer = HW_AllocMailboxBuffer(128); buffer->MbxHeader.Length = 29;
    ((BYTE *)buffer)[33] = 0x56; ((BYTE *)buffer)[34] = 0x89;
    CHECK(MBX_MailboxSendReq(buffer, 0) == 0);
    CHECK(sent[33] == 0x56 && sent[34] == 0x89 && sent[35] == 0xa5); Recover();
    Start(35); buffer = HW_AllocMailboxBuffer(128); buffer->MbxHeader.Length = 28;
    ((BYTE *)buffer)[32] = 0x12; ((BYTE *)buffer)[33] = 0x34;
    CHECK(MBX_MailboxSendReq(buffer, 0) == 0);
    CHECK(sent[32] == 0x12 && sent[33] == 0x34 && sent[34] == 0 && sent[35] == 0xa5); Recover();
}
static void ImpossibleSend(void)
{
    unsigned size;
    for (size = 0; size < 10; size++) {
        TMBX *buffer;
        Start(128); u16SendMbxSize = (UINT16)size;
        buffer = HW_AllocMailboxBuffer(128); buffer->MbxHeader.Length = 128;
        CHECK(MBX_MailboxSendReq(buffer, COE_SERVICE | FRAGMENTS_FOLLOW) == 0);
        CHECK(!writes && !u8MailboxSendReqStored && !pCoeSendStored);
        Recover();
    }
}
static void NoMemory(void)
{
    TMBX *all[24];
    unsigned i;
    Start(128);
    for (i = 0; i < 24; i++) all[i] = HW_AllocMailboxBuffer(128);
    bNoMbxMemoryAvailable = TRUE;
    HW_FreeMailboxBuffer(all[23]);
    memset(receive, 0, 128); receive[0] = 128;
    MBX_CheckAndCopyMailbox(); CHECK(writes == 1); MbxError(7);
    for (i = 0; i < 23; i++) HW_FreeMailboxBuffer(all[i]);
    Recover();
}
static void ListsAndRepeat(void)
{
    const unsigned char counts[10] = {16,0,1,0,1,0,0,0,0,0};
    const unsigned char first[20] = {0,0x10,1,0x10,8,0x10,9,0x10,10,0x10,
        0x18,0x10,0xf1,0x10,0,0x16,0,0x1a,0,0x1c};
    const unsigned char last[12] = {0x12,0x1c,0x13,0x1c,0x32,0x1c,0x33,0x1c,0,0x60,0,0x70};
    unsigned char saved[34];
    Start(34); Request(8, 8, 1, 0, 0);
    CHECK(sent[0] == 18 && sent[8] == 2 && sent[10] == 0);
    CHECK(memcmp(sent + 14, counts, 10) == 0); Recover();
    Start(34); Request(8, 8, 1, 1, 0);
    CHECK(sent[0] == 28 && sent[8] == 0x82 && sent[10] == 1 && sent[11] == 0);
    CHECK(memcmp(sent + 14, first, 20) == 0); memcpy(saved, sent, 34);
    MBX_MailboxReadInd();
    CHECK(writes == 2 && sent[0] == 18 && sent[8] == 2 && sent[10] == 0);
    CHECK(memcmp(sent + 12, last, 12) == 0);
    CHECK(!nSdoInfoFragmentsLeft && !u8MailboxSendReqStored);
    MBX_MailboxRepeatReq(); CHECK(writes == 3 && memcmp(sent, saved, 34) == 0);
    MBX_MailboxReadInd(); CHECK(writes == 4 && memcmp(sent + 12, last, 12) == 0);
    Recover();
}
int main(int argc, char **argv)
{
    SYSTEM_INFO systemInfo;
    DWORD previous;
    unsigned i;
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    GetSystemInfo(&systemInfo);
    for (i = 0; i < 24; i++) {
        pages[i] = VirtualAlloc(NULL, 2 * systemInfo.dwPageSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        CHECK(pages[i]);
        CHECK(VirtualProtect(pages[i] + systemInfo.dwPageSize, systemInfo.dwPageSize, PAGE_NOACCESS, &previous));
        pool[i] = (TMBX *)(pages[i] + systemInfo.dwPageSize - 128);
    }
    CHECK(COE_ObjDictionaryInit() == 0);
    if (argc == 1 || strcmp(argv[1], "ingress") == 0) Ingress();
    if (argc == 1 || strcmp(argv[1], "names") == 0) Names();
    if (argc == 1 || strcmp(argv[1], "short") == 0) ShortHeaders();
    if (argc == 1 || strcmp(argv[1], "list") == 0) InvalidList();
    if (argc == 1 || strcmp(argv[1], "send") == 0) SendBounds();
    if (argc == 1 || strcmp(argv[1], "minimum") == 0) NameWire(34);
    if (argc == 1 || strcmp(argv[1], "maximum") == 0) MaximumSend();
    if (argc == 1 || strcmp(argv[1], "impossible") == 0) ImpossibleSend();
    if (argc == 1 || strcmp(argv[1], "nomemory") == 0) NoMemory();
    if (argc == 1 || strcmp(argv[1], "continuation") == 0) ListsAndRepeat();
    COE_ClearObjDictionary();
    for (i = 0; i < 24; i++) CHECK(VirtualFree(pages[i], 0, MEM_RELEASE));
    printf("SSC real mailbox length chain: %u checks passed\n", checks);
    return 0;
}
