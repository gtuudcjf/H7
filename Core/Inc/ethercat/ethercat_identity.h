#ifndef ETHERCAT_IDENTITY_H
#define ETHERCAT_IDENTITY_H

#include <stdint.h>

/* Lab-only Vendor ID: not ETG-assigned and not for product releases. */
#define ETHERCAT_DEVICE_NAME        "H743-LAN9252-HBI-LAB"
#define ETHERCAT_VENDOR_ID          UINT32_C(0x00F0C743)
#define ETHERCAT_PRODUCT_CODE       UINT32_C(0x00009252)
#define ETHERCAT_REVISION           UINT32_C(2)
#define ETHERCAT_SERIAL_NUMBER      UINT32_C(0x00000001)
#define ETHERCAT_SII_SIZE_BYTES     UINT16_C(2048)

#endif /* ETHERCAT_IDENTITY_H */
