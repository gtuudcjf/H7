# Minimal SSC 5.11 core for H743 / LAN9252

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

The vendor application include `el9800appl.h` in three sources is an explicit
Task 6 integration point. Task 6 replaces it with the actual H743 application
header/dictionary; it is not copied from the F407 example. Task 7 supplies
the port implementations. Full firmware compilation/linkage follows those
tasks and Keil integration in Task 8.
