/* Full vendor mailbox implementation with a tracking fixed-pool/ESC boundary. */
#include <stdio.h>
#include <stdlib.h>
#include "mailbox.c"
#include "ecatcoe.c"

static TMBX pool[24];
static unsigned owned[24], released[24], checks, dispatches, frees;
static TSYNCMAN syncManagers[2];
UINT8 nAlStatus = STATE_PREOP;
volatile UINT16 u16dummy;
UINT16 nSdoInfoFragmentsLeft;
MEM_ADDR aSdoInfoHeader[GET_MEM_SIZE(SDO_INFO_HEADER_BYTE_SIZE)];

#define CHECK(expr) do { checks++; if (!(expr)) { \
    fprintf(stderr, "SSC mailbox failure line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)
void *HW_AllocMailboxBuffer(UINT16 size)
{
    unsigned i;
    CHECK(size <= 128);
    for (i = 0; i < 24; i++) if (!owned[i]) { owned[i] = 1; return &pool[i]; }
    return NULL;
}
void HW_FreeMailboxBuffer(void *pointer)
{
    unsigned i;
    if (!pointer || pointer == &asMbx[0] || pointer == &asMbx[1]) return;
    for (i = 0; i < 24; i++) if (pointer == &pool[i]) {
        CHECK(owned[i] == 1);
        owned[i] = 0; released[i]++; frees++; return;
    }
    CHECK(0);
}
void HW_EscRead(MEM_ADDR *data, UINT16 address, UINT16 length) { memset(data, 0, length); }
void HW_EscWrite(MEM_ADDR *data, UINT16 address, UINT16 length) { }
void DisableSyncManChannel(UINT8 channel) { }
void EnableSyncManChannel(UINT8 channel) { }
TSYNCMAN *GetSyncMan(UINT8 channel) { return &syncManagers[channel]; }
UINT8 SDOS_SdoInd(TINITSDOMBX *data) { dispatches++; HW_FreeMailboxBuffer(data); return 0; }
UINT8 SDOS_SdoInfoInd(TSDOINFORMATION *data) { dispatches++; HW_FreeMailboxBuffer(data); return 0; }

int main(void)
{
    unsigned i;
    TMBX *buffers[24];
    MBX_Init();
    for (i = 0; i < 9; i++) {
        buffers[i] = HW_AllocMailboxBuffer(128);
        buffers[i]->MbxHeader.Flags[0] = MBX_TYPE_COE << MBX_SHIFT_TYPE;
    }
    for (i = 0; i < 3; i++) {
        CHECK(PutInMbxQueue(buffers[i], &sMbxReceiveQueue) == 0);
        CHECK(PutInMbxQueue(buffers[i+3], &sMbxSendQueue) == 0);
    }
    psReadMbx = buffers[6]; psRepeatMbx = buffers[7]; psStoreMbx = buffers[6];
    psWriteMbx = buffers[0]; pCoeSendStored = buffers[8];
    nSdoInfoFragmentsLeft = 7;
    bMbxRunning = TRUE;
    MBX_StopMailboxHandler();
    CHECK(!bMbxRunning);
    CHECK(sMbxReceiveQueue.firstInQueue == sMbxReceiveQueue.lastInQueue);
    CHECK(sMbxSendQueue.firstInQueue == sMbxSendQueue.lastInQueue);
    CHECK(frees == 9);
    for (i = 0; i < 9; i++) CHECK(owned[i] == 0 && released[i] == 1);
    CHECK(psReadMbx == NULL && psRepeatMbx == NULL && psStoreMbx == NULL && psWriteMbx == NULL && pCoeSendStored == NULL);
    CHECK(nSdoInfoFragmentsLeft == 0);
    MBX_Main(); CHECK(dispatches == 0);
    MBX_Init(); MBX_Main(); CHECK(dispatches == 0);
    syncManagers[0].PhysicalStartAddress = 0x1000; syncManagers[0].Length = 128;
    syncManagers[1].PhysicalStartAddress = 0x1080; syncManagers[1].Length = 128;
    CHECK(MBX_StartMailboxHandler() == 0);
    buffers[0] = HW_AllocMailboxBuffer(128);
    memset(buffers[0], 0, 128);
    buffers[0]->MbxHeader.Flags[0] = MBX_TYPE_COE << MBX_SHIFT_TYPE;
    ((TCOEMBX *)buffers[0])->CoeHeader = COESERVICE_SDOREQUEST << COEHEADER_COESERVICESHIFT;
    CHECK(PutInMbxQueue(buffers[0], &sMbxReceiveQueue) == 0);
    MBX_Main(); CHECK(dispatches == 1);
    for (i = 0; i < 24; i++) { buffers[i] = HW_AllocMailboxBuffer(128); CHECK(buffers[i] != NULL); }
    CHECK(HW_AllocMailboxBuffer(128) == NULL);
    for (i = 0; i < 24; i++) HW_FreeMailboxBuffer(buffers[i]);
    /* Pending CoE may alias a retained response: stop must free once. */
    psReadMbx = HW_AllocMailboxBuffer(128); pCoeSendStored = psReadMbx;
    MBX_StopMailboxHandler(); CHECK(pCoeSendStored == NULL && psReadMbx == NULL);
    printf("SSC mailbox lifecycle: %u checks passed\n", checks);
    return 0;
}
