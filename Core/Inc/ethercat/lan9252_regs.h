#ifndef LAN9252_REGS_H
#define LAN9252_REGS_H

#include <stdint.h>

/* LAN9252 DS00001909A, HBI system/EtherCAT registers. */
#define LAN9252_REG_ID_REV          0x0050u
#define LAN9252_REG_BYTE_TEST       0x0064u
#define LAN9252_REG_HW_CFG          0x0074u
#define LAN9252_REG_CSR_DATA        0x0300u
#define LAN9252_REG_CSR_CMD         0x0304u
#define LAN9252_REG_PRAM_RD_ADDR_LEN 0x0308u
#define LAN9252_REG_PRAM_RD_CMD      0x030Cu
#define LAN9252_REG_PRAM_WR_ADDR_LEN 0x0310u
#define LAN9252_REG_PRAM_WR_CMD      0x0314u
#define LAN9252_REG_PRAM_RD_FIFO     0x0004u
#define LAN9252_REG_PRAM_WR_FIFO     0x0020u
#define LAN9252_BUSY                UINT32_C(0x80000000)
#define LAN9252_CSR_READ            UINT32_C(0x40000000)
#define LAN9252_PRAM_ABORT          UINT32_C(0x40000000)
#define LAN9252_PRAM_AVAILABLE      UINT32_C(0x00000001)
#define LAN9252_PRAM_COUNT_MASK     UINT32_C(0x00001F00)
#define LAN9252_HW_READY            UINT32_C(0x08000000)
#define LAN9252_BYTE_TEST_VALUE     UINT32_C(0x87654321)
#define LAN9252_CHIP_ID             0x9252u
#define LAN9252_ESC_PDI_CONTROL     0x0140u
#define LAN9252_ESC_EEPROM_STATUS   0x0502u
#define LAN9252_ESC_INFORMATION     0x0000u
#define LAN9252_ESC_AL_STATUS       0x0130u
#define LAN9252_PDI_HBI             0x89u
/* Busy, write/command error, not loaded, and configuration checksum error. */
#define LAN9252_EEPROM_INVALID      0xF800u
#define LAN9252_PRAM_START          0x1000u
#define LAN9252_PRAM_END            0x2000u

#endif
