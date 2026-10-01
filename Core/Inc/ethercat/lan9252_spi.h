#ifndef LAN9252_SPI_H
#define LAN9252_SPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LAN9252_OK = 0,
    LAN9252_ERR_ARGUMENT,
    LAN9252_ERR_IO,
    LAN9252_ERR_DIRECT_TIMEOUT,
    LAN9252_ERR_BYTE_TEST,
    LAN9252_ERR_READY_TIMEOUT,
    LAN9252_ERR_CHIP_ID,
    LAN9252_ERR_PDI,
    LAN9252_ERR_CSR_TIMEOUT,
    LAN9252_ERR_PRAM_TIMEOUT,
    LAN9252_ERR_EEPROM
} LAN9252Status;

/* Synchronous full-duplex transport. NULL tx clocks zero bytes; NULL rx
 * discards incoming bytes. Must return within timeout_us, using ERR_IO for
 * bus errors and ERR_DIRECT_TIMEOUT for a timeout. It must not control CS.
 * time_us must keep advancing during polling and wrap naturally at 32 bits.
 * Calls on the same controller must be serialized by the caller (no ISR use).
 */
typedef LAN9252Status (*LAN9252TransferFn)(void *context,
    const uint8_t *tx, uint8_t *rx, size_t length, uint32_t timeout_us);
typedef struct {
    LAN9252TransferFn transfer;
    void (*select)(void *context);
    void (*deselect)(void *context);
    uint32_t (*time_us)(void *context);
    void *context;
} LAN9252Transport;
typedef struct {
    LAN9252Transport transport;
    LAN9252Status last_error;
    uint32_t timeout_count;
} LAN9252Context;
typedef struct {
    uint32_t byte_test;
    uint32_t id_rev;
    uint32_t hw_cfg;
    uint8_t pdi_control;
    uint16_t eeprom_status;
    uint8_t esc_type;
    uint8_t esc_revision;
    uint16_t esc_build;
    uint16_t al_status;
} LAN9252ProbeResult;

/* Microsecond deadlines, always below half the timebase wrap interval. */
#define LAN9252_DIRECT_TIMEOUT_US UINT32_C(1000)
#define LAN9252_CSR_TIMEOUT_US    UINT32_C(10000)
#define LAN9252_PRAM_TIMEOUT_US   UINT32_C(10000)
#define LAN9252_READY_TIMEOUT_US  UINT32_C(100000)

/* Init drives CS idle when a deselect callback is supplied. All public calls
 * record their final status, with one timeout_count increment per failed call.
 * Zero lengths, NULL buffers, wraparound and out-of-range ESC accesses fail
 * before bus activity. ESC requests must fit wholly within CSR [0,0x1000) or
 * PRAM [0x1000,0x2000). Buffers may have any byte alignment. On failure a read
 * buffer may be partially updated; callers must discard it. A write may have
 * partially reached the device. Probe clears its result and retains fields
 * read before an error for diagnostics; it never writes ESC configuration.
 */
void LAN9252_InitContext(LAN9252Context *ctx, const LAN9252Transport *transport);
LAN9252Status LAN9252_ReadDirect(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length);
LAN9252Status LAN9252_WriteDirect(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length);
LAN9252Status LAN9252_ReadEsc(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length);
LAN9252Status LAN9252_WriteEsc(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length);
LAN9252Status LAN9252_Probe(LAN9252Context *ctx, LAN9252ProbeResult *result);

#ifdef __cplusplus
}
#endif
#endif
