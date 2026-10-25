#ifndef ETHERCAT_BUILD_H
#define ETHERCAT_BUILD_H
#ifndef ETHERCAT_BRINGUP_ONLY
#define ETHERCAT_BRINGUP_ONLY 1
#endif
#ifndef ETHERCAT_MOTOR_CONTROL
#define ETHERCAT_MOTOR_CONTROL 0
#endif
#ifndef ETHERCAT_MOTOR_OPEN_ANGLE
#define ETHERCAT_MOTOR_OPEN_ANGLE 0
#endif
/* Stage B default. Setting 1 is an explicit operator-reviewed rebuild. */
#ifndef ETHERCAT_MOTOR_POWER_ENABLED
#define ETHERCAT_MOTOR_POWER_ENABLED 0
#endif
#if (ETHERCAT_BRINGUP_ONLY != 0 && ETHERCAT_BRINGUP_ONLY != 1) || \
    (ETHERCAT_MOTOR_CONTROL != 0 && ETHERCAT_MOTOR_CONTROL != 1) || \
    (ETHERCAT_MOTOR_OPEN_ANGLE != 0 && ETHERCAT_MOTOR_OPEN_ANGLE != 1) || \
    (ETHERCAT_MOTOR_POWER_ENABLED != 0 && ETHERCAT_MOTOR_POWER_ENABLED != 1)
#error Application_mode_flags_must_be_boolean
#endif
#if ETHERCAT_BRINGUP_ONLY && ETHERCAT_MOTOR_CONTROL
#error EtherCAT_bringup_and_motor_modes_are_mutually_exclusive
#endif
#if ETHERCAT_MOTOR_POWER_ENABLED && !ETHERCAT_MOTOR_CONTROL
#error Power_gate_requires_EtherCAT_motor_mode
#endif
#if ETHERCAT_MOTOR_OPEN_ANGLE && !ETHERCAT_MOTOR_CONTROL
#error Open_angle_requires_EtherCAT_motor_mode
#endif
#endif
