# EtherCAT motor speed implementation ledger and plan

Approved specification: the user's 2026-10-16 implementation request in this conversation.
Goal: revision-2 custom PDO speed/enable control, using existing local motor loops.
No firmware download, EEPROM write, hardware actuation, remote push or merge.

## Tasks

- [x] 1. Pure protocol/state machine: 8/20-byte little-endian PDO, zero-command
  rearm, fresh sequence, 100 ms lease, 2 s startup deadline, reset and stop priority.
- [x] 2. Independent command guard and motor adapter: 1 kHz TIM6, physical
  gate-off first, foreground stop, race-safe start/PWM permit, coherent feedback.
- [x] 3. Revision-2 dictionary/identity, SSC sizes, main-loop mode and dedicated
  Keil project; preserve revision-1 and original local motor behavior.
- [x] 4. Motor ESI/SII candidate generator, PLC source/project and bench runbook.
- [x] 5. Software regression tests, full ARM builds, independent review and handoff.

## Acceptance

Host tests must exercise real core behavior, malformed wire data, sequence wrap,
duplicate/reversed sequence, stop on stale data, startup interruption and loss
of authority. Build all three firmware modes. Default motor project is physically
power-locked. Hardware timing <=110 ms and actual rotation remain operator gates.

## Execution record

- Baseline 60f6444, branch codex/ethercat-motor-speed. Existing EtherCAT suite
  (0 failed/0 pending) and motor suite (8 checks) passed before implementation.
- Ruling: use requested dedicated branch in current checkout, no new worktree:
  user did not request a worktree; preserve dirty .uvprojx/.uvoptx and references.
- Pre-flight: task 1 wire lengths feed task 3 dictionary and task 4 ESI/PLC.
  One fixed 8-byte Rx / 20-byte Tx ABI is authoritative across all three.
- Pre-flight: task 2 independent IRQ guard may revoke task 1 start authority.
  Revocation is latched and checked at every physical enable, not only in main.
- Ruling: default motor firmware power gate remains locked. Hardware stages B-E
  cannot be certified by host tests and must not be reported as completed.
- Ruling: create separate Keil project rather than overwrite user's dirty project.
  Original user files remain untouched; new project carries the new target.

## Software progress and verification

- Task 1: complete: actual pure core, 106 checks; initial absent-source RED then GREEN.
- Task 2: complete: guard 19 checks, hardware adapter 15 checks in each power build,
  bridge 4 locked / 22 permitted checks. Pending interrupt simulation covers enable/revoke
  ordering; stalled startup and OP loss tested against actual safety adapter.
- Task 3: complete: real SSC dictionary/object access 47 checks. ARM preprocessing proves
  motor startup isolation, telemetry exclusion, and conflicting build defines rejected.
  All four full ARMCC5 targets compiled with zero warnings/errors before final fixes.
- Found/fixed motor fault reset admission: full Stop preserves fault_code but changes
  mode to STOPPED. Policy regression RED (missing admission) -> GREEN; reset now accepts
  stopped+latched, forbids active modes, preserves fault if hardware Stop fails.
- PLC reset contract RED -> GREEN: wait phase 80 for slave fault acknowledgement rather
  than immediately returning to idle against stale PDO feedback. XML contract test is
  explicitly not a substitute for real ST compilation.
- Task 4: ESI and 2048-byte SII candidate generated; CRC/identity/SM/signed mapping tests
  pass. No hardware validation or write authorization. Chinese runbook in EtherCAT/motor.
  Minimal PLC sources delivered; isolated offline XAE import/build passed, then exported
  as a native tpzip, stripped of automatic boot payload/path traversal entries and
  re-imported/recompiled successfully. Direct handcrafted .plcproj is inventory/template
  only, not the supported import route.
- Keil parser finding: XmlDocument serialization caused import exit 15; preserve original
  raw project formatting and substitute only build fields. Separate project now builds.
- XAE automation finding: COM tree handles must not travel through the PowerShell output
  pipeline (IEnumVARIANT unrolls them). Keep explicit script-scope handles. Import an
  existing PLC project with CreateChild subtype 1, not template subtype 0.
- Fresh final ARM builds and full regressions passed; hardware stages B-E NOT run.
- Ruling: retain uncommitted implementation for handoff; user requested implementation,
  not a commit/merge/push in this turn. Cost: must explicitly stage new files before commit.

## Review focus

Review interrupted startup and physical enable races (PC4/MOE), stale/duplicate/reversed
sequence and reset/rearm, OP-loss callbacks and SDO bypass, motor fault cleanup/reset,
coherent feedback, PLC reset/stop/link gating, exact ESI/SII/SSC/PLC wire identity/lengths,
preserved revision-1/local modes, and limits of host-only evidence. No hardware flashing.

## Final independent review and one fix pass

- Initial Astra reviewer failed before reviewing due to quota. Sole working fresh-context
  reviewer used available gpt-6.1-sol. Read-only result: no Critical, four Important,
  no Minor. No second review commissioned.
- Final: fixed CH4 startup MOE reassertion — added the shared, serialized sampling-PWM
  permit entry. Native safety adapter regression RED (missing entry) -> GREEN for both
  power builds; pending revoke during HAL start clears MOE. Actual MotorControl_Start
  is extracted/compiled by cleanup tests and traverses this entry. ARM builds green.
- Final: fixed interrupted three-zero preparation — exact 0/1/0/0/1 fresh-frame sequence
  RED -> GREEN; enabled frames while DISARMED cancel preparation.
- Final: fixed receive-before-poll lease renewal — READY at 20 ms, Enable at 150 ms
  RED -> GREEN; expiry checked before accepting renewal. No start action after expiry.
- Final: fixed lost cleanup state — actual production Start/Stop/ClearFault functions,
  generated mechanically into a native test harness, RED -> GREEN. PWM CH4, base timer,
  ADC stop failure each preserves pending cleanup and fault until retry succeeds.
  Additional same-root startup rollback test RED -> GREEN: attempted ADC startup also
  records pending cleanup before later startup steps can fail.
- Final: Ruling: reviewer deferred all hardware behavior (cutoff latency, real rotation,
  ISR load/timing, EEPROM device compatibility and bench acceptance) — do not claim it;
  all remain B-E operator gates — cost: physical acceptance and independent power cutoff
  still required before any powered test.
- Final: Ruling: deliver XAE-native tpzip as supported PLC import route; keep handcrafted
  .plcproj explicitly labeled inventory/template only — real source compilation and
  sanitized archive re-import passed in XAE 3.1.4024.15 — cost: other XAE versions still
  require their own import/build and variable-link checks.
- Final evidence: core 114, guard 19, safety 21 per power build, bridge 4/22, SSC dictionary
  47, runtime policy and actual startup/cleanup regression passed. Legacy EtherCAT suite
  0 failed / 0 pending; motor telemetry suite 8 checks; ARM mode isolation passed;
  Python artifacts 5 passed; git diff --check clean (line-ending notices only).
- Final evidence: four fresh ARMCC5 full builds 0 errors / 0 warnings. PLC delivered
  archive offline re-import/build passed: tmp/plc_verify_8cf3626ddac1490182a735bc6552ffaf.
  No activation, login, download or EEPROM write. No commit, push or merge.
