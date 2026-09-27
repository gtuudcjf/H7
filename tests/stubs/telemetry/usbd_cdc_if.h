#ifndef TELEMETRY_TEST_CDC_H
#define TELEMETRY_TEST_CDC_H
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;
uint8_t CDC_Transmit_FS(uint8_t *, uint16_t);
#endif
