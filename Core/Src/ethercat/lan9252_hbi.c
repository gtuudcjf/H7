#include "lan9252_hbi.h"
#include "lan9252_regs.h"
#include <string.h>

volatile LAN9252FirstFault g_lan9252_first_fault;
static uint32_t diagnostic_esc_address = UINT32_C(0xffffffff);

static uint32_t load32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void store32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24);
}
static int valid_context(const LAN9252Context *ctx)
{
    return ctx && ctx->transport.read16 && ctx->transport.write16 &&
           ctx->transport.finish_dword && ctx->transport.time_us;
}
static LAN9252Status finish(LAN9252Context *ctx, LAN9252Status status)
{
    if (ctx) {
        ctx->last_error = status;
        if (status == LAN9252_ERR_DIRECT_TIMEOUT || status == LAN9252_ERR_CSR_TIMEOUT ||
            status == LAN9252_ERR_PRAM_TIMEOUT || status == LAN9252_ERR_READY_TIMEOUT)
            ++ctx->timeout_count;
    }
    return status;
}
void LAN9252_InitContext(LAN9252Context *ctx, const LAN9252Transport *transport)
{
    if (!ctx) return;
    if (transport) ctx->transport = *transport;
    else memset(&ctx->transport, 0, sizeof(ctx->transport));
    ctx->timeout_count = 0;
    ctx->last_error = valid_context(ctx) ? LAN9252_OK : LAN9252_ERR_ARGUMENT;
}
static uint32_t now(LAN9252Context *ctx)
{
    return ctx->transport.time_us(ctx->transport.context);
}
static int expired(LAN9252Context *ctx, uint32_t start, uint32_t timeout)
{
    return (uint32_t)(now(ctx) - start) >= timeout;
}
/* Each internal DWORD is assembled by exactly two physical halfwords.
 * The callbacks own volatile MMIO/barriers; no byte-pointer aliases reach FMC. */
static LAN9252Status direct(LAN9252Context *ctx, uint16_t address,
                           uint8_t *rx, const uint8_t *tx, size_t length)
{
    uint32_t start = now(ctx), phase = 1, reason = 0;
    uint16_t words[2] = {0,0};
    LAN9252Status status = LAN9252_OK;
    unsigned i;
    (void)length;
    if (tx) { words[0] = (uint16_t)load32(tx); words[1] = (uint16_t)(load32(tx) >> 16); }
    for (i = 0; i < 2; ++i) {
        phase = i + 1;
        if (expired(ctx, start, LAN9252_DIRECT_TIMEOUT_US)) {
            status = LAN9252_ERR_DIRECT_TIMEOUT; reason = 1; break;
        }
        status = rx ? ctx->transport.read16(ctx->transport.context,
                         (uint16_t)(address + 2U*i), &words[i]) :
                      ctx->transport.write16(ctx->transport.context,
                         (uint16_t)(address + 2U*i), words[i]);
        if (status != LAN9252_OK) {
            if (status != LAN9252_ERR_DIRECT_TIMEOUT) status = LAN9252_ERR_IO;
            reason = 2; break;
        }
        if (expired(ctx, start, LAN9252_DIRECT_TIMEOUT_US)) {
            status = LAN9252_ERR_DIRECT_TIMEOUT; reason = 3; break;
        }
    }
    ctx->transport.finish_dword(ctx->transport.context);
    if (status == LAN9252_OK && expired(ctx, start, LAN9252_DIRECT_TIMEOUT_US)) {
        status = LAN9252_ERR_DIRECT_TIMEOUT; reason = 3;
    }
    if (status == LAN9252_OK && rx) store32(rx,(uint32_t)words[0] | ((uint32_t)words[1] << 16));
    if (status != LAN9252_OK && !g_lan9252_first_fault.valid) {
        g_lan9252_first_fault.address = address;
        g_lan9252_first_fault.esc_address = diagnostic_esc_address;
        g_lan9252_first_fault.write = tx != NULL;
        g_lan9252_first_fault.phase = phase;
        g_lan9252_first_fault.reason = reason;
        g_lan9252_first_fault.status = (uint32_t)status;
        g_lan9252_first_fault.elapsed_us = (uint32_t)(now(ctx) - start);
        g_lan9252_first_fault.budget_us = LAN9252_DIRECT_TIMEOUT_US;
        g_lan9252_first_fault.valid = 1;
    }
    return status;
}
static int valid_direct(LAN9252Context *ctx, uint16_t address, const void *data, size_t length)
{
    return valid_context(ctx) && data && length == 4u &&
           !(address & 3u) && address < 0x400u;
}
LAN9252Status LAN9252_ReadDirect(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length)
{
    return finish(ctx, valid_direct(ctx,address,data,length) ?
                  direct(ctx,address,data,NULL,length) : LAN9252_ERR_ARGUMENT);
}
LAN9252Status LAN9252_WriteDirect(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length)
{
    return finish(ctx, valid_direct(ctx,address,data,length) ?
                  direct(ctx,address,NULL,data,length) : LAN9252_ERR_ARGUMENT);
}
static LAN9252Status read32(LAN9252Context *ctx, uint16_t address, uint32_t *value)
{
    uint8_t b[4];
    LAN9252Status status = direct(ctx,address,b,NULL,4);
    if (status == LAN9252_OK) *value = load32(b);
    return status;
}
static LAN9252Status write32(LAN9252Context *ctx, uint16_t address, uint32_t value)
{
    uint8_t b[4]; store32(b,value);
    return direct(ctx,address,NULL,b,4);
}
/* Poll both before and after the transfer: a late ready bit cannot turn a
 * missed operation deadline into a success. Transfer errors retain their type. */
static LAN9252Status poll(LAN9252Context *ctx, uint16_t reg, int fifo,
                          uint32_t start, uint32_t timeout, LAN9252Status timeout_status)
{
    for (;;) {
        uint32_t value;
        LAN9252Status status;
        if (expired(ctx,start,timeout)) return timeout_status;
        status = read32(ctx,reg,&value);
        if (status != LAN9252_OK) return status;
        if (expired(ctx,start,timeout)) return timeout_status;
        if (fifo) {
            if ((value & LAN9252_PRAM_AVAILABLE) && (value & LAN9252_PRAM_COUNT_MASK))
                return LAN9252_OK;
        } else if (!(value & LAN9252_BUSY)) return LAN9252_OK;
    }
}
static LAN9252Status csr(LAN9252Context *ctx, uint16_t address,
                         uint8_t *rx, const uint8_t *tx, size_t length)
{
    uint32_t start = now(ctx);
    while (length) {
        uint8_t b[4] = {0,0,0,0};
        size_t n = (!(address & 3u) && length >= 4) ? 4u :
                   (!(address & 1u) && length >= 2) ? 2u : 1u;
        uint32_t command = LAN9252_BUSY | (rx ? LAN9252_CSR_READ : 0) |
                           ((uint32_t)n << 16) | address;
        LAN9252Status status = poll(ctx,LAN9252_REG_CSR_CMD,0,start,
                                    LAN9252_CSR_TIMEOUT_US,LAN9252_ERR_CSR_TIMEOUT);
        if (status != LAN9252_OK) return status;
        if (tx) {
            memcpy(b,tx,n);
            status = direct(ctx,LAN9252_REG_CSR_DATA,NULL,b,4);
            if (status != LAN9252_OK) return status;
        }
        status = write32(ctx,LAN9252_REG_CSR_CMD,command);
        if (status != LAN9252_OK) return status;
        status = poll(ctx,LAN9252_REG_CSR_CMD,0,start,LAN9252_CSR_TIMEOUT_US,LAN9252_ERR_CSR_TIMEOUT);
        if (status != LAN9252_OK) return status;
        if (rx) {
            status = direct(ctx,LAN9252_REG_CSR_DATA,b,NULL,4);
            if (status != LAN9252_OK) return status;
            memcpy(rx,b,n); rx += n;
        } else tx += n;
        length -= n; address = (uint16_t)(address + n);
    }
    return expired(ctx,start,LAN9252_CSR_TIMEOUT_US) ? LAN9252_ERR_CSR_TIMEOUT : LAN9252_OK;
}
static LAN9252Status pram(LAN9252Context *ctx, uint16_t address,
                          uint8_t *rx, const uint8_t *tx, size_t length)
{
    uint16_t cmd = rx ? LAN9252_REG_PRAM_RD_CMD : LAN9252_REG_PRAM_WR_CMD;
    uint16_t addr_len = rx ? LAN9252_REG_PRAM_RD_ADDR_LEN : LAN9252_REG_PRAM_WR_ADDR_LEN;
    uint16_t fifo = rx ? LAN9252_REG_PRAM_RD_FIFO : LAN9252_REG_PRAM_WR_FIFO;
    size_t offset = address & 3u;
    uint32_t start = now(ctx);
    LAN9252Status status = write32(ctx,cmd,LAN9252_PRAM_ABORT);
    if (status != LAN9252_OK) return status;
    status = poll(ctx,cmd,0,start,LAN9252_PRAM_TIMEOUT_US,LAN9252_ERR_PRAM_TIMEOUT);
    if (status != LAN9252_OK) return status;
    status = write32(ctx,addr_len,((uint32_t)length << 16) | address);
    if (status != LAN9252_OK) return status;
    status = write32(ctx,cmd,LAN9252_BUSY);
    if (status != LAN9252_OK) return status;
    while (length) {
        uint8_t b[4] = {0,0,0,0};
        size_t n = length < 4u - offset ? length : 4u - offset;
        /* Consume just one DWORD per availability poll. This also handles
         * transfers larger than the 16-DWORD FIFO without an unchecked burst. */
        status = poll(ctx,cmd,1,start,LAN9252_PRAM_TIMEOUT_US,LAN9252_ERR_PRAM_TIMEOUT);
        if (status != LAN9252_OK) return status;
        if (rx) {
            status = direct(ctx,fifo,b,NULL,4);
            if (status != LAN9252_OK) return status;
            memcpy(rx,b+offset,n); rx += n;
        } else {
            memcpy(b+offset,tx,n);
            status = direct(ctx,fifo,NULL,b,4);
            if (status != LAN9252_OK) return status;
            tx += n;
        }
        length -= n; offset = 0;
    }
    return poll(ctx,cmd,0,start,LAN9252_PRAM_TIMEOUT_US,LAN9252_ERR_PRAM_TIMEOUT);
}
static LAN9252Status esc(LAN9252Context *ctx, uint16_t address,
                         uint8_t *rx, const uint8_t *tx, size_t length)
{
    LAN9252Status status;
    size_t end = address < LAN9252_PRAM_START ? LAN9252_PRAM_START : LAN9252_PRAM_END;
    if (!valid_context(ctx) || (!rx && !tx) || !length || address >= end || length > end - address)
        return LAN9252_ERR_ARGUMENT;
    diagnostic_esc_address = address;
    status = address < LAN9252_PRAM_START ? csr(ctx,address,rx,tx,length) : pram(ctx,address,rx,tx,length);
    diagnostic_esc_address = UINT32_C(0xffffffff);
    return status;
}
LAN9252Status LAN9252_ReadEsc(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length)
{
    return finish(ctx,esc(ctx,address,data,NULL,length));
}
LAN9252Status LAN9252_WriteEsc(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length)
{
    return finish(ctx,esc(ctx,address,NULL,data,length));
}
static LAN9252Status probe(LAN9252Context *ctx, LAN9252ProbeResult *r)
{
    uint32_t start;
    uint8_t b[4];
    LAN9252Status status;
    if (!valid_context(ctx) || !r) return LAN9252_ERR_ARGUMENT;
    memset(r,0,sizeof(*r));
    start = now(ctx);
    do {
        status = read32(ctx,LAN9252_REG_BYTE_TEST,&r->byte_test);
        if (status != LAN9252_OK) return status;
        if (expired(ctx,start,LAN9252_READY_TIMEOUT_US)) return LAN9252_ERR_BYTE_TEST;
        if (r->byte_test == LAN9252_BYTE_TEST_VALUE) break;
    } while (1);
    start = now(ctx);
    for (;;) {
        if (expired(ctx,start,LAN9252_READY_TIMEOUT_US)) return LAN9252_ERR_READY_TIMEOUT;
        status = read32(ctx,LAN9252_REG_HW_CFG,&r->hw_cfg);
        if (status != LAN9252_OK) return status;
        if (expired(ctx,start,LAN9252_READY_TIMEOUT_US)) return LAN9252_ERR_READY_TIMEOUT;
        if (r->hw_cfg & LAN9252_HW_READY) break;
    }
    status = read32(ctx,LAN9252_REG_ID_REV,&r->id_rev);
    if (status != LAN9252_OK) return status;
    if ((r->id_rev >> 16) != LAN9252_CHIP_ID) return LAN9252_ERR_CHIP_ID;
    status = esc(ctx,LAN9252_ESC_PDI_CONTROL,&r->pdi_control,NULL,1);
    if (status != LAN9252_OK) return status;
    if (r->pdi_control != LAN9252_PDI_HBI) return LAN9252_ERR_PDI;
    status = esc(ctx,LAN9252_ESC_EEPROM_STATUS,b,NULL,2);
    if (status != LAN9252_OK) return status;
    r->eeprom_status = (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    if (r->eeprom_status & LAN9252_EEPROM_INVALID) return LAN9252_ERR_EEPROM;
    status = esc(ctx,LAN9252_ESC_INFORMATION,b,NULL,4);
    if (status != LAN9252_OK) return status;
    r->esc_type = b[0]; r->esc_revision = b[1];
    r->esc_build = (uint16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    status = esc(ctx,LAN9252_ESC_AL_STATUS,b,NULL,2);
    if (status != LAN9252_OK) return status;
    r->al_status = (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    return LAN9252_OK;
}
LAN9252Status LAN9252_Probe(LAN9252Context *ctx, LAN9252ProbeResult *result)
{
    return finish(ctx,probe(ctx,result));
}
