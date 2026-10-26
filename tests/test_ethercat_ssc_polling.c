/* Runs production SM functions and MainLoop's timer block extracted by the
 * integration runner. No runtime hardware/application stubs ship in firmware. */
#include <stdio.h>
#include <stdlib.h>
#include "el9800hw.h"
#include "esc.h"

static UINT32 nowUs, clockStep, sscTimerBaseline;
static unsigned reads, writes, ticks, timeouts, checks;
static UINT16 readValue, writeValue, lastAddress;
static UINT8 timeoutChannel;
static BOOL fault, faultOnWrite, faultOnRead;

#define CHECK(expr) do { checks++; if (!(expr)) { \
    fprintf(stderr, "SSC polling failure line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)

UINT32 HW_GetTimer(void) { UINT32 value = nowUs; nowUs += clockStep; return value; }
BOOL HW_AccessFault(void) { return fault; }
void HW_RecordSyncManTimeout(UINT8 channel) { timeouts++; timeoutChannel = channel; fault = TRUE; }
void HW_EscRead(MEM_ADDR *data, UINT16 address, UINT16 length)
{
    /* Fail rather than hang if a regression restores the unbounded vendor loop. */
    if (++reads > 2048) { fprintf(stderr, "Unbounded SyncManager poll\n"); exit(1); }
    CHECK(length == 2 && address == lastAddress);
    memcpy(data, &readValue, 2);
    if (faultOnRead) fault = TRUE;
}
void HW_EscWrite(MEM_ADDR *data, UINT16 address, UINT16 length)
{
    CHECK(length == 2);
    memcpy(&writeValue, data, 2);
    lastAddress = address;
    writes++;
    if (faultOnWrite) fault = TRUE;
}
void ECAT_CheckTimer(void) { ticks++; }
#define HW_ClearTimer() (nowUs = 0)
#include "ssc_polling_bodies.h"

static void Reset(void)
{
    nowUs = clockStep = sscTimerBaseline = 0;
    reads = writes = ticks = timeouts = 0;
    readValue = writeValue = lastAddress = 0;
    fault = faultOnWrite = faultOnRead = FALSE;
}

int main(void)
{
    Reset(); readValue = SM_SETTING_PDI_DISABLE;
    DisableSyncManChannel(2);
    CHECK(reads == 1 && writes == 1 && timeouts == 0);
    CHECK(writeValue == SM_SETTING_PDI_DISABLE && lastAddress == 0x816);
    Reset(); EnableSyncManChannel(3);
    CHECK(reads == 1 && writes == 1 && writeValue == 0 && lastAddress == 0x81e);

    Reset(); clockStep = 250;
    DisableSyncManChannel(1);
    CHECK(timeouts == 1 && timeoutChannel == 1 && fault && reads <= 5);
    Reset(); nowUs = 0xffffff00U; clockStep = 250; readValue = SM_SETTING_PDI_DISABLE;
    EnableSyncManChannel(3);
    CHECK(timeouts == 1 && timeoutChannel == 3 && fault && reads <= 5);
    Reset(); DisableSyncManChannel(0);
    CHECK(timeouts == 1 && reads == 64 && fault);

    Reset(); fault = TRUE; EnableSyncManChannel(0);
    CHECK(writes == 0 && reads == 0 && timeouts == 0);
    Reset(); faultOnWrite = TRUE; DisableSyncManChannel(0);
    CHECK(writes == 1 && reads == 0 && timeouts == 0);
    Reset(); faultOnRead = TRUE; readValue = SM_SETTING_PDI_DISABLE;
    DisableSyncManChannel(0);
    CHECK(writes == 1 && reads == 1 && fault && timeouts == 0);

    Reset(); sscTimerBaseline = 50000; nowUs = 50999; PollTimer();
    CHECK(ticks == 0 && sscTimerBaseline == 50000);
    nowUs = 51000; PollTimer(); CHECK(ticks == 1 && sscTimerBaseline == 51000);
    nowUs = 53750; PollTimer(); CHECK(ticks == 3 && sscTimerBaseline == 53000);
    Reset(); sscTimerBaseline = 0xfffffe00U; nowUs = 488; PollTimer();
    CHECK(ticks == 1 && sscTimerBaseline == 488);
    Reset(); nowUs = 20500; PollTimer();
    CHECK(ticks == 8 && sscTimerBaseline == 8000);
    PollTimer(); CHECK(ticks == 16 && sscTimerBaseline == 16000);
    PollTimer(); CHECK(ticks == 20 && sscTimerBaseline == 20000);
    PollTimer(); CHECK(ticks == 20 && sscTimerBaseline == 20000);
    printf("SSC polling: %u checks passed\n", checks);
    return 0;
}
