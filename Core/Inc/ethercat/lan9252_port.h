#ifndef LAN9252_PORT_H
#define LAN9252_PORT_H
#include "lan9252_spi.h"

typedef struct {
    LAN9252ProbeResult probe;
    LAN9252Status last_error;
    uint32_t timeout_count;
    uint8_t syncman_timeout_channel;
} LAN9252PortObservations;

/* Read-only foreground observations. HW declarations live in el9800hw.h.
 * One controller, serialized foreground calls only, including ISR aliases.
 * HW_Init clears the sticky fault but preserves outstanding pool ownership.
 * HW_Release resets the pool only after the caller has drained the SSC stack. */
const LAN9252PortObservations *LAN9252_PortGetObservations(void);
#endif
