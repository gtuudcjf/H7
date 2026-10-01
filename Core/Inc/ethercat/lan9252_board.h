#ifndef LAN9252_BOARD_H
#define LAN9252_BOARD_H
#include "lan9252_spi.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Call after SystemClock_Config, MX_GPIO_Init and MX_SPI1_Init. Serialized
 * foreground use only, with the normal 1 ms HAL tick running. Fixed CPU clock.
 * DWT counts through polling; debug halt/sleep is outside the timing contract.
 * Stock blocking HAL has 1 ms timeout granularity: a remaining budget below
 * 1 ms may overrun by less than 1 ms plus interrupt/adapter execution overhead.
 * Pre/post checks still reject late completion, without retrying a failed call.
 */
const LAN9252Transport *LAN9252_BoardTransport(void);
#ifdef __cplusplus
}
#endif
#endif
