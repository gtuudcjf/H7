#ifndef LAN9252_HBI_H
#define LAN9252_HBI_H
#include <stddef.h>
#include <stdint.h>

typedef enum {
    LAN9252_OK = 0, LAN9252_ERR_ARGUMENT, LAN9252_ERR_IO,
    LAN9252_ERR_DIRECT_TIMEOUT, LAN9252_ERR_BYTE_TEST,
    LAN9252_ERR_READY_TIMEOUT, LAN9252_ERR_CHIP_ID, LAN9252_ERR_PDI,
    LAN9252_ERR_CSR_TIMEOUT, LAN9252_ERR_PRAM_TIMEOUT,
    LAN9252_ERR_EEPROM, LAN9252_ERR_SM_TIMEOUT, LAN9252_ERR_CLOCK
} LAN9252Status;

/* Serialized foreground only. Offsets are BYTE offsets, not word indices.
 * Each callback performs one bounded halfword access. finish_dword completes
 * writes and waits >=1us before another DWORD, including failed pairs.
 * time_us is monotonic modulo 2^32 and continues during polling. */
typedef struct {
    LAN9252Status (*read16)(void *context, uint16_t offset, uint16_t *value);
    LAN9252Status (*write16)(void *context, uint16_t offset, uint16_t value);
    void (*finish_dword)(void *context);
    uint32_t (*time_us)(void *context);
    void *context;
} LAN9252Transport;
typedef struct {
    LAN9252Transport transport;
    LAN9252Status last_error;
    uint32_t timeout_count;
} LAN9252Context;
typedef struct {
    uint32_t byte_test, id_rev, hw_cfg;
    uint8_t pdi_control;
    uint16_t eeprom_status;
    uint8_t esc_type, esc_revision;
    uint16_t esc_build, al_status;
} LAN9252ProbeResult;
typedef struct {
    uint32_t valid, address, esc_address, write, phase, reason, status;
    uint32_t elapsed_us, budget_us;
} LAN9252FirstFault;
extern volatile LAN9252FirstFault g_lan9252_first_fault;

#define LAN9252_DIRECT_TIMEOUT_US UINT32_C(1000)
#define LAN9252_CSR_TIMEOUT_US UINT32_C(10000)
#define LAN9252_PRAM_TIMEOUT_US UINT32_C(10000)
#define LAN9252_READY_TIMEOUT_US UINT32_C(100000)

/* Physical accesses require length=4, DWORD alignment, offset<0x400.
 * ESC buffers may have any alignment; requests must fit wholly in CSR
 * [0,0x1000) or PRAM [0x1000,0x2000). Discard partial reads on any error.
 * A partial write may have reached hardware. Port faults are sticky until
 * explicit initialization; first physical fault persists until MCU reset.
 * Probe performs reads first, then CSR commands; it never writes EEPROM. */
void LAN9252_InitContext(LAN9252Context *ctx, const LAN9252Transport *transport);
LAN9252Status LAN9252_ReadDirect(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length);
LAN9252Status LAN9252_WriteDirect(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length);
LAN9252Status LAN9252_ReadEsc(LAN9252Context *ctx, uint16_t address, uint8_t *data, size_t length);
LAN9252Status LAN9252_WriteEsc(LAN9252Context *ctx, uint16_t address, const uint8_t *data, size_t length);
LAN9252Status LAN9252_Probe(LAN9252Context *ctx, LAN9252ProbeResult *result);
#endif
