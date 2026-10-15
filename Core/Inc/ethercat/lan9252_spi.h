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
    LAN9252_ERR_EEPROM,
    LAN9252_ERR_SM_TIMEOUT
} LAN9252Status;

/* Synchronous full-duplex transport. NULL tx clocks zero bytes; NULL rx
 * discards incoming bytes. Must bound calls by timeout_us, using ERR_IO for
 * bus errors and ERR_DIRECT_TIMEOUT for a timeout. It must not control CS.
 * time_us must keep advancing during polling and wrap naturally at 32 bits.
 * A blocking HAL adapter may document its minimum timeout granularity; the
 * H743 board adapter permits <2 ms extra wait for its compensated Tick guard,
 * plus execution/interrupt overhead, and reports late completion as timeout.
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

/* First failed physical SPI access since MCU startup. Diagnostic only: never
 * cleared by InitContext or a later success. Read only when valid == 1.
 * phase: 1 command/header, 2 data. reason: 1 deadline before transport,
 * 2 transport failure, 3 deadline after successful transport.
 * esc_address is the logical ESC request, or 0xffffffff for a direct probe.
 * The separate board HAL snapshot is absent for driver-only deadlines. */
typedef struct {
    uint32_t valid;
    uint32_t address, esc_address, command, phase, reason, status;
    uint32_t elapsed_us, budget_us;
} LAN9252FirstFault;
extern volatile LAN9252FirstFault g_lan9252_first_fault;

/* Microsecond deadlines, always below half the timebase wrap interval. */
#define LAN9252_DIRECT_TIMEOUT_US UINT32_C(1000)
#define LAN9252_CSR_TIMEOUT_US    UINT32_C(10000)
#define LAN9252_PRAM_TIMEOUT_US   UINT32_C(10000)
#define LAN9252_READY_TIMEOUT_US  UINT32_C(100000)

/* Init drives CS idle when a deselect callback is supplied. All public calls
 * record their final status, with one timeout_count increment per failed call.
 * Direct accesses require length == 4, a DWORD-aligned physical address below
 * 0x4000, and thus zero SPI address-mode bits [15:14]. This API does not accept
 * partial DWORDs or multi-register transfers (DS00001909A sections 10.2/10.2.6.1).
 * Invalid lengths/addresses and NULL buffers fail before bus activity.
 * ESC requests may have arbitrary nonzero byte lengths and must fit wholly
 * within CSR [0,0x1000) or PRAM [0x1000,0x2000).
 * Buffers may have any byte alignment. On failure a read
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
