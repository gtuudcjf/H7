# Mode 2 application profile on the existing revision-2 PDO layout

This is an experimental custom profile, not CiA402. No new EEPROM/ESI install is required.
Vendor/Product/Revision remain 00F0C743/00009252/2; Rx/Tx remain 8/20 bytes, little-endian.
Existing 13 GVL link variables retain their names, directions, types and offsets.

## Compatibility gate

Use only with firmware built using `ETHERCAT_MOTOR_OPEN_ANGLE=1` and the matching MAIN.
The original `EtherCATMotorSpeedDemo` is the encoder-speed PLC application, not a different PC/PLC device.
Never run it as a mode-2 controller. A profile marker/acknowledgement prevents cross-pairing from starting PWM.
The original speed build still requires valid encoder calibration and rejects the new acknowledgement bit.

| ControlWord | Meaning |
| --- | --- |
| 0x0100 | Profile confirmed, disabled; TargetSpeed must be zero for arming |
| 0x0101 | Profile confirmed, enable; first rising edge requires zero target |
| 0x0180 | Profile confirmed, fault reset edge; enable=0, target=0 |

Unknown control bits reject the command. Disable has priority even on duplicate/invalid commands.
Before a first acknowledged frame, unacknowledged disabled zero frames cannot arm or refresh the heartbeat.
After profile acceptance, losing the acknowledgement latches fault 11. SDO is not a command/heartbeat path.
Three advancing disabled-zero sequences are required before a new zero-target enable edge.

## StatusWord

| Bit | Meaning |
| --- | --- |
| 0 | Start request can be accepted; not proof of ADC calibration or torque |
| 1 | Local mode-2 control is running; not proof the rotor follows |
| 2 | Fault latched |
| 3 | Effective OP exchange |
| 4 | Fresh advancing command sequence |
| 5 | Disarmed/rearm required |
| 6 | Starting, including ADC calibration and alignment |
| 7 | Complete measured speed feedback valid: always 0 in this mode |
| 8 | Mode-2 application profile marker |
| 9 | Fresh, finite mode-2 current/control snapshot valid |
| 10 | Full electrical stop complete, driver EN low and TIM8 MOE low |

Control snapshot validity also expires when the last successful ADC sample is older than 2 ms,
independently of the PWM tick-age counter. Reading feedback never refreshes that timestamp.

Do not compare the whole word with the old idle value 56. Extra profile/output-off bits are intentional.
For example, healthy mode-2 READY with outputs off is normally 0x0519 (1305).

`TargetSpeed`: equivalent magnetic-field motor-shaft speed, 0.1 rpm, range [-200,200].
`ActiveTargetSpeed`: active frequency converted to the same equivalent units; not rotor feedback.
`ActualSpeed`: zero placeholder with speed-valid bit false; never interpret as actual standstill.
`ActualIq`: q-axis current in the virtual electrical frame, mA; not measured rotor-frame torque current.
`CycleCounter`: application iteration while local control runs; no substitute for command heartbeat.
`AcceptedSequence`: accepted advancing command number. Repeats do not refresh freshness, normal uint32 wrap works.

## Faults and shutdown

Existing codes 0-10 remain unchanged (not OP=1, SPI=2, command=3, heartbeat=4, power locked=9).
New codes: profile mismatch=11; experiment time limit=12; direct reverse=13; stop timeout=14.
SPI latch is not remotely resettable. Other resets require disabled zero command and stopped hardware.

Mode-2 defaults: alignment/run current 0.1 A, reference-vector ceiling 0.3 A, voltage ceiling 0.15 pu.
Additional current fault: any phase absolute value >0.6 A for 3 consecutive valid samples.
These are unverified candidate bench settings, not an instantaneous current guarantee.
Changing the field direction in an active session faults; stop, confirm mechanical standstill, then start anew.
Commanding zero after motion ramps field frequency to zero, then current references to zero, then turns output off.
After alignment, a zero-only wait lasts at most 500 ms. Zero frequency is not permission to hold DC indefinitely.
An independent TIM6 check limits an enabled session to 5 s and first-nonzero motion to 3 s, whichever expires first.
Fresh commands, zero demands and repeated requests cannot extend the same session.
Faults/disable/100 ms heartbeat loss revoke output immediately; the foreground completes peripheral cleanup.
Electrical shutdown is not mechanical instant stop, certified STO, or evidence of measured <=110 ms latency.
