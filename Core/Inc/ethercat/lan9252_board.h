#ifndef LAN9252_BOARD_H
#define LAN9252_BOARD_H
#include "lan9252_spi.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Call after SystemClock_Config, MX_GPIO_Init and MX_SPI1_Init. Serialized
 * foreground use only, with the normal 1 ms HAL tick running. Fixed CPU clock.
 * DWT counts through polling; debug halt/sleep is outside the timing contract.
 * HAL's integer-tick guard is ceil(remaining_us / 1000) + 1 ms so a Tick
 * boundary cannot prematurely abort a valid transfer. Blocking can overrun
 * the remaining budget by <2 ms plus interrupt/adapter execution overhead.
 * Pre/post checks still reject late completion, without retrying a failed call.
 */
const LAN9252Transport *LAN9252_BoardTransport(void);

/* One-shot lab comparison, not a replacement for the normal driver probe.
 * HAL status: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT; 0xffffffff = not called.
 * value_valid means the HAL calls succeeded, not that the value is correct. */
typedef struct {
    uint32_t value;
    uint32_t value_valid;
    uint32_t hal_calls;
    uint32_t hal_status[2];
    uint32_t hal_error[2];
    uint8_t tx[8];
    uint8_t rx[8];
} LAN9252SpiReadDebug;

typedef struct {
    uint32_t done;
    uint32_t passed_mask; /* bit0 split, bit1 fast single, bit2 normal single */
    LAN9252SpiReadDebug fast_split;
    LAN9252SpiReadDebug fast_single;
    LAN9252SpiReadDebug normal_single;
} LAN9252SpiDebug;

extern volatile LAN9252SpiDebug g_lan9252_spi_debug;

/* First board-adapter failure since MCU startup; valid is published last.
 * reason: 1 budget before HAL, 2 HAL_TIMEOUT, 3 HAL_ERROR with EOT/FLAG,
 * 4 budget after HAL, 5 other HAL error/busy.
 * hal_status/error == 0xffffffff means HAL was not called for this chunk.
 * Register/count snapshots are AFTER the HAL return/cleanup, not raw flags
 * at the instant of failure. Never read RXDR/TXDR or clear peripheral flags.
 * Unchanged by the one-shot BYTE_TEST comparison and later successes. */
typedef struct {
    uint32_t valid, reason, hal_status, hal_error, hal_state;
    uint32_t elapsed_us, timeout_us, timeout_ms, chunk_length;
    uint32_t spi_sr, spi_cr1, spi_cr2, spi_cfg1, spi_cfg2;
    uint32_t tx_remaining, rx_remaining;
} LAN9252HalFault;
extern volatile LAN9252HalFault g_lan9252_hal_fault;
/* After SPI1 init, before servicing SSC. Reads BYTE_TEST only, once per call.
 * Serialized foreground only; never pause inside a CS-low transaction. */
void LAN9252_BoardRunSpiDiagnostics(void);
#ifdef __cplusplus
}
#endif
#endif
