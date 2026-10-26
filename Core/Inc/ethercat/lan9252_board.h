#ifndef LAN9252_BOARD_H
#define LAN9252_BOARD_H
#include "lan9252_hbi.h"
/* After SystemClock_Config, MX_GPIO_Init and MX_FMC_Init. No ISR callers.
 * Fixed 480MHz CPU/240MHz FMC. DWT must run, with the normal HAL tick.
 * Debug halt/sleep is outside the timeout contract. Returns NULL if the
 * unlocked DWT counter fails its bounded startup progress check. */
const LAN9252Transport *LAN9252_BoardTransport(void);
#endif
