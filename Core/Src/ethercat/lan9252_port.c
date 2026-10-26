#include "lan9252_port.h"
#include "lan9252_board.h"
#include "el9800hw.h"
#include <string.h>

static LAN9252Context controller;
static LAN9252PortObservations observations;
static union { uint32_t alignment; uint8_t bytes[128]; } mailbox_pool[24];
static uint8_t mailbox_owned[24];

static void RecordStatus(LAN9252Status status)
{
    if (observations.last_error == LAN9252_OK && status != LAN9252_OK)
        observations.last_error = status;
    observations.timeout_count = controller.timeout_count;
}

const LAN9252PortObservations *LAN9252_PortGetObservations(void)
{
    return &observations;
}

UINT8 HW_Init(void)
{
    const LAN9252Transport *transport;
    memset(&observations, 0, sizeof(observations));
    observations.syncman_timeout_channel = 0xff;
    transport = LAN9252_BoardTransport();
    LAN9252_InitContext(&controller, transport);
    RecordStatus(transport ? controller.last_error : LAN9252_ERR_CLOCK);
    if (!HW_AccessFault()) RecordStatus(LAN9252_Probe(&controller, &observations.probe));
    return HW_AccessFault() ? 1 : 0;
}

void HW_Release(void)
{
    /* Caller must drain retained/queued SSC buffers before releasing the pool. */
    memset(mailbox_owned, 0, sizeof(mailbox_owned));
}

BOOL HW_AccessFault(void) { return observations.last_error != LAN9252_OK; }

void HW_RecordSyncManTimeout(UINT8 channel)
{
    observations.syncman_timeout_channel = channel;
    RecordStatus(LAN9252_ERR_SM_TIMEOUT);
}

UINT32 HW_GetTimer(void)
{
    return controller.transport.time_us ?
        controller.transport.time_us(controller.transport.context) : 0;
}

void HW_EscRead(MEM_ADDR *pData, UINT16 Address, UINT16 Len)
{
    LAN9252Status status;
    if (!pData || !Len) status = LAN9252_ERR_ARGUMENT;
    else status = LAN9252_ReadEsc(&controller, Address, (uint8_t *)pData, Len);
    if (status != LAN9252_OK && pData && Len) memset(pData, 0, Len);
    RecordStatus(status);
}

void HW_EscWrite(MEM_ADDR *pData, UINT16 Address, UINT16 Len)
{
    RecordStatus(!pData || !Len ? LAN9252_ERR_ARGUMENT :
        LAN9252_WriteEsc(&controller, Address, (const uint8_t *)pData, Len));
}

UINT16 HW_GetALEventRegister(void)
{
    uint8_t data[2];
    HW_EscRead(data, 0x0220, sizeof(data));
    return (UINT16)((UINT16)data[0] | ((UINT16)data[1] << 8));
}

void *HW_AllocMailboxBuffer(UINT16 size)
{
    unsigned i;
    if (size < 10 || size > 128) return NULL;
    for (i = 0; i < 24; ++i) {
        if (!mailbox_owned[i]) {
            mailbox_owned[i] = 1;
            return mailbox_pool[i].bytes;
        }
    }
    return NULL;
}

void HW_FreeMailboxBuffer(void *pointer)
{
    unsigned i;
    if (!pointer) return;
    for (i = 0; i < 24; ++i) {
        if (pointer == mailbox_pool[i].bytes) {
            mailbox_owned[i] = 0;
            return;
        }
    }
}
