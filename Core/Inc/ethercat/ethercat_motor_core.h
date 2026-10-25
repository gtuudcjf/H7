#ifndef ETHERCAT_MOTOR_CORE_H
#define ETHERCAT_MOTOR_CORE_H
#include <stddef.h>
#include <stdint.h>

#define EMC_RX_BYTES 8U
#define EMC_TX_BYTES 20U
#define EMC_HEARTBEAT_MS 100U
#define EMC_START_TIMEOUT_MS 2000U
#define EMC_SPEED_LIMIT_D10 200
#define EMC_ACTION_START 1U
#define EMC_ACTION_STOP 2U
#define EMC_ACTION_CLEAR 4U
#define EMC_ACTION_FEED 8U
#define EMC_STATUS_READY 1U
#define EMC_STATUS_RUNNING 2U
#define EMC_STATUS_FAULT 4U
#define EMC_STATUS_OP 8U
#define EMC_STATUS_FRESH 16U
#define EMC_STATUS_REARM 32U
#define EMC_STATUS_STARTING 64U
#define EMC_STATUS_VALID 128U
#define EMC_CONTROL_OPEN_ANGLE_ACK 256U
#define EMC_STATUS_OPEN_ANGLE 256U
#define EMC_STATUS_CONTROL_VALID 512U
#define EMC_STATUS_OUTPUT_OFF 1024U

typedef enum { EMC_PROFILE_SPEED=0, EMC_PROFILE_OPEN_ANGLE=1 } EMC_Profile;

typedef enum {
    EMC_FAULT_NONE=0, EMC_FAULT_NOT_OP=1, EMC_FAULT_SPI=2,
    EMC_FAULT_COMMAND=3, EMC_FAULT_HEARTBEAT=4, EMC_FAULT_SEQUENCE=5,
    EMC_FAULT_START=6, EMC_FAULT_START_TIMEOUT=7, EMC_FAULT_MOTOR=8,
    EMC_FAULT_POWER_LOCKED=9, EMC_FAULT_STOP=10, EMC_FAULT_PROFILE=11,
    EMC_FAULT_RUNTIME=12, EMC_FAULT_REVERSE=13, EMC_FAULT_STOP_TIMEOUT=14
} EMC_Fault;
typedef enum { EMC_DISARMED=0, EMC_READY, EMC_STARTING, EMC_RUNNING } EMC_State;
typedef struct {
    int16_t speed_d10, iq_ma, active_target_d10;
    uint16_t motor_fault;
    uint8_t calibrated, running, valid;
    uint8_t start_permitted, control_valid, output_off, stop_requested;
} EMC_Feedback;
typedef struct {
    EMC_Feedback feedback;
    EMC_Profile profile;
    uint8_t profile_seen;
    EMC_State state;
    EMC_Fault fault;
    uint32_t accepted_sequence, last_command_ms, start_ms, cycle_counter;
    int16_t target_speed_d10;
    uint16_t previous_control;
    uint8_t link, spi_ok, sequence_valid, fresh, zeros, actions;
} EMC_Core;

void EMC_Init(EMC_Core *c);
void EMC_InitProfile(EMC_Core *c, EMC_Profile profile);
void EMC_SetLink(EMC_Core *c, uint8_t active, uint8_t spi_ok);
void EMC_SetFeedback(EMC_Core *c, const EMC_Feedback *feedback);
void EMC_Receive(EMC_Core *c, const uint8_t *wire, size_t size, uint32_t now_ms);
void EMC_Poll(EMC_Core *c, uint32_t now_ms);
void EMC_Fail(EMC_Core *c, EMC_Fault fault);
uint8_t EMC_TakeActions(EMC_Core *c);
void EMC_ActionResult(EMC_Core *c, uint8_t action, uint8_t success, uint32_t now_ms);
void EMC_Pack(const EMC_Core *c, uint8_t wire[EMC_TX_BYTES]);
#endif
