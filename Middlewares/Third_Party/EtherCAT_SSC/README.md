# Minimal SSC 5.11 core for H743 / LAN9252

## HBI integration (2026-10-24)

This checkout imports the seven reviewed core sources without changes from
H7 commit `60f64449e8fa3eea2335b9b3200a759ef26bce98`
(`codex/lan9252-first-op`). The source hashes in
`tests/ssc_source_hashes.json` pin that provenance independently of Git history.
The historical SPI import notes below remain applicable to those SSC patches.

The application/port/service are reused with a new FMC16 single-phase HBI
transport. `CONTROLLER_32BIT=1` describes the Cortex-M7 C data model, not the
physical parallel bus width. Physical accesses are ordered halfword pairs;
the port still presents byte-addressed buffers to SSC. Identity is now
H743-LAN9252-HBI-LAB revision 2; CoE/FreeRun/PDO sizes are retained.
No SOES source or F407 hardware driver is linked into this firmware.

Imported from the repository reference
`407 LAN9252 Ethercat spi/STM32F407 Ethercat/Ethercat/{Inc,src}`.
Import resumed and verified on 2026-10-02. The original Beckhoff SSC 5.11
author/version/change-history headers are retained. `el9800hw.h` is a new,
project-owned compatibility header, replacing the reference hardware header.

The retained implementation consists of exactly seven sources: `ecatappl.c`,
`ecatcoe.c`, `ecatslv.c`, `mailbox.c`, `objdef.c`, `sdoserv.c`, and `coeappl.c`.
The ten imported headers are `applInterface.h`, `coeappl.h`, `ecat_def.h`,
`ecatappl.h`, `ecatcoe.h`, `ecatslv.h`, `esc.h`, `mailbox.h`, `objdef.h`, and
`sdoserv.h`. All other reference material is excluded, including TcHelper,
test/sample/CiA402 applications, FoE/EoE/AoE/diagnosis, F407 `el9800hw.c`,
temporary files, DLLs, and LIBs. The reference tree is read-only input.

Attribution is to the original author `EthercatSSC@beckhoff.com` and the
version notices in each imported file. No standalone license file was found
in that reference Ethercat subtree; this import does not replace upstream
terms or assert a new license for vendor code. Keep the original notices
with subsequent imports. 后续升级必须重新做集成测试 (every later upgrade must
repeat the integration tests).

## Configuration and local patches

`ecat_def.h` selects polling FreeRun, CoE and mailbox support, little-endian
32-bit controller types from `<stdint.h>`, byte-addressed ESC memory, packed
mailbox/general wire structures, and the vendor DWORD-aligned object rules.
PDO sizes are Rx 2 bytes / Tx 6 bytes; mailbox size is 128 bytes. Identity
macros consume `Core/Inc/ethercat/ethercat_identity.h`.

AL events, DC, timer/ESC interrupts, default main, mailbox queue mode,
segmented SDO, Complete Access, deferred SDO responses, emergency, diagnosis,
VoE/AoE/SoE/EoE/FoE, bootstrap, and ESC EEPROM access are disabled.
Ordinary expedited/normal single-entry SDO and SDOInfo remain supported.
There are no calls to the C heap. Generic `ALLOCMEM`/`FREEMEM` defaults are
null/no-op defensive fallbacks for disabled segmented paths; active mailbox
allocations use the port's fixed pool hooks.

The supplied generated reference omitted several original feature guards:
setting `MAILBOX_QUEUE=0` still leaves its finite mailbox rings and allocation
lifecycle in the compiled sources. Those are preserved. `sdoserv.c` now
honors disabled Complete Access with abort `0x06010004`, rejects segmented
requests with unsupported-access abort `0x06010000` before allocation/state
changes, and converts a single-entry handler's deferred `ABORTIDX_WORKING`
result into the same unsupported-access abort. SDOInfo fragmentation retains
the vendor continuation path and uses mailbox pool buffers.

Single-entry SDO validates the configured subindex before reading a record
descriptor or calculating its length; missing subindices abort `0x06090011`.
Downloads must match the entry's byte width before generic or custom writes:
short and long transfers abort `0x06070013` and `0x06070012`, respectively.
`objdef.c` also validates direct single-entry writes at its public boundary.
Configured PDO mapping entries remain writable while subindex zero is
cleared. The Complete Access check applies only to initiation commands;
both segment toggle values retain the unsupported-segment abort.

`mailbox.c` drains every entry in both queues on stop, releases a separately
retained CoE response without freeing retained-pointer aliases twice, clears
write/retained pointers, and resets CoE continuation state before restart.
`objdef.c` uses pointer-width `uintptr_t` for the existing odd-address ENUM
alignment check and masked-address reconstruction; byte ESC storage is not
used as an address-sized integer.

Final review length fixes (2026-10-03) validate mailbox payloads against both
the configured SM capacity minus the six-byte header and the 128-byte pool
allocation, before addition/copy/dispatch and at the physical send boundary.
CoE/SDO/SDOInfo headers are checked before field access; unsupported SDOInfo
list types return a bounded error. Oversized responses become INVALIDSIZE
mailbox errors through existing queue/repeat ownership, cancelling their CoE
continuation; impossible error capacities release the request without I/O.
The no-memory error transfers ownership to the send path, and the last-word
trigger preserves terminal bytes at exact/odd SM ends. Record-name size-only
and copy queries now agree, so names that cannot fit SM34 are omitted while
the advertised minimum 34/default 128-byte mailbox sizes remain unchanged.
`ecatcoe.c` includes `ecatslv.h` for the existing byte-order macros.
`tests/test_ethercat_ssc_lengths.c` exercises the real mailbox/CoE/SDOInfo,
dictionary and application chain using protected pool blocks and bounded ESC
accesses; no protocol dispatcher or mailbox sender is substituted.

The same generated-reference issue affected synchronization:
`coeappl.c` now advertises SM/DC/subcycle support only under its feature
flags. `ecatslv.c` selects FreeRun for both PDO directions when AL events
are disabled, rejects user-requested non-FreeRun with AL status `0x0028`,
and rejects active/auto-active DC or Sync signals with AL status `0x0030`
when DC support is disabled. Without these guards the generated reference
would select SM synchronization despite the polling configuration.

`ecatappl.c` initializes a separate timer baseline in `MainInit`. `MainLoop`
uses unsigned elapsed microseconds and consumes at most eight 1 ms ticks per
call, advancing the baseline only by consumed ticks. It retains fractional
time and backlog across calls and never resets the monotonic clock. The
unsigned interval assumes the loop is serviced within one 32-bit microsecond
wrap (approximately 71.6 minutes). Existing SM/DC cycle measurement is not
used in this FreeRun configuration.

`ecatslv.c` bounds each SyncManager enable/disable acknowledgement wait by
1,000 elapsed microseconds and 64 reads, whichever occurs first. A latched
access fault stops the wait, including faults from the initiating write or
an acknowledgement read. Timeout records the channel through the port and
must latch the fault. The complete duration also includes the bounded port
transaction time and its accepted HAL timeout granularity; this is not an
exact submillisecond scheduling guarantee.

## Required Task 7 hardware interface

Task 6 binds `ecatappl.c`, `ecatslv.c`, and `coeappl.c` to the project-owned
`ethercat_app.h`, replacing their three example application includes.
The fixed application dictionary is mandatory; `coeappl.c` registers it
unconditionally while retaining the registration result/error handling.
`ecatcoe.c` includes `el9800hw.h` so its active mailbox allocator call has
the real pointer-returning declaration in each independent translation unit.
These are the only Task 6 vendor source deltas. No warning suppression or
forced include is required to compile these changed units with ARMClang.

`Inc/el9800hw.h` directly declares the interface using SSC types, and does
not depend on an unfinished port header. Task 7 implements these declarations
over `lan9252_port`; its implementations should include this header to avoid
prototype drift. No runtime stubs are shipped.

- `UINT8 HW_Init(void)`, `void HW_Release(void)`.
- `UINT16 HW_GetALEventRegister(void)` returns the ESC AL-event word.
- `void HW_EscRead(MEM_ADDR *, UINT16, UINT16)` and the corresponding
  `HW_EscWrite` use byte addresses and byte lengths. Handle unaligned SSC
  buffers safely, zero failed reads, and latch transport faults. The ISR
  compatibility names map to these functions; interrupts remain disabled.
- `UINT32 HW_GetTimer(void)` returns monotonic microseconds modulo 2^32.
- `BOOL HW_AccessFault(void)` reports a sticky fault, cleared only during
  explicit initialization/recovery.
- `void HW_RecordSyncManTimeout(UINT8 channel)` records the timeout channel
  and latches that fault so the service can stop stack processing.
- `void *HW_AllocMailboxBuffer(UINT16 size)` and
  `void HW_FreeMailboxBuffer(void *pointer)` manage a bounded static pool.

For this imported reference's `MAX_MBX_QUEUE_SIZE=10`, reserve **24 blocks
of 128 bytes**, each aligned to at least four bytes (3,072 bytes total).
This is a conservative capacity bound: 10 receive-ring entries + 10
send-ring entries + at most two distinct read/repeat/backup buffers + one
pending `pCoeSendStored` buffer + one in-flight request/SDOInfo allocation.
Read/repeat/backup pointers alias during repeat exchange, so there are at
most two distinct retained buffers across those three pointer roles. Normal
serial CoE traffic generally retains only two or three blocks; the capacity
bound intentionally also covers the preserved rings.

Requests are 10 bytes for mailbox errors, the configured receive size up
to 128 bytes, or `SIZEOF(TMBX)` (128) for SDOInfo continuation. Return NULL
for oversize/exhaustion; no heap fallback. Blocks remain owned until the
SSC explicitly frees them, potentially across service calls and mailbox
repeat requests; never clear/reuse a live block on a clock tick or after
`HW_EscWrite`. INIT/stop frees retained buffers and drains queues. Freeing
NULL is a no-op. The vendor also owns `asMbx[2]` statically; pool free must
not treat these addresses as pool allocations. Pool reinitialization is
allowed only after the stack is stopped and its references are discarded.

## Verification and integration boundary

Run `powershell -ExecutionPolicy Bypass -File
tests/test_ethercat_ssc_integration.ps1`. It checks the import/configuration
contracts and executes real SyncManager bodies, the real `MainLoop` timer
block, the real synchronization selection and object initialization, and
the full SDO dispatcher against fake ESC/clock/dictionary/mailbox
boundaries. Additional tests compile the actual full mailbox/CoE and SDO/
object-access sources, track fixed-pool stop/restart ownership, and guard a
bounded descriptor table with an inaccessible page. Focused tests prove
wrap, backlog, finite stalled-clock polling,
fault handling, feature aborts, ordinary SDO reads/writes, and preserved
SDOInfo fragmentation through the bounded pool interface. Polling extraction
does not verify the rest of `MainLoop` or firmware linkage.

The real application tests compile the full `coeappl.c`, `objdef.c`, and
`sdoserv.c` with the actual H743 dictionary and PDO adapter. They exercise
identity/mapping values and widths through OBJ_Read and SDO uploads,
read-only map/assignment enforcement, writable ControlWord, unaligned
two/six-byte wire mapping, and start/stop/error/fault resets. Counting runs
only during active input/output exchange after valid PDO reception, and
counts FreeRun application iterations rather than master cycles.
Task 7 supplies the port implementations. Full firmware compilation/linkage
follows that task and Keil integration in Task 8.

Task 9 ARMCC compatibility patches: TMBX storage is explicitly four-byte
aligned without changing its 128-byte wire size or packed fields. The HW
pool also provides four-byte-aligned storage. SDO word pointers use the
mailbox byte base plus checked even wire offsets (upload 12/16, normal
download 16, SDOInfo first/count 14 and continuation 12); compile-time
checks enforce the layout. This avoids ARMCC's incompatible __packed
member pointers without suppressing diagnostics. Disabled segment helper
bodies are excluded with SEGMENTED_SDO_SUPPORTED. Four redundant equality
parentheses and an unreachable FreeRun break are removed in objdef.c.
Imported C/header trailing whitespace and blank EOF lines are mechanically
normalized; original notices and text are preserved. Real SDOInfo tests
check count/first/continuation byte placement and response sentinels.
