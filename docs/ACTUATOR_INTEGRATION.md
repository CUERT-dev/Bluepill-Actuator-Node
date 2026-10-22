# Mapping and driver repair evidence

Validated locally on 2026-10-20. Scope: mapping and motor policy libraries,
their build registration, and host tests. No CAN receiver, RTOS task, physical
adapter, pin assignment or hardware activation was added.

## Branch and contract

The initial checkout was `feature/gp-actuator-integration` at `40ccd80`, tracking
`origin/feature/gp-actuator-integration-4-`. It lacked the libraries.
GitHub `ls-remote` confirmed Maintaining at
`b77a3c24822c4722dc4f9ab325edbb86ea09073c`, matching the local remote-tracking ref.
Its history contains:

- PR #5: head `b9332a7ad23bfee93d11c1d3456c6d439647cc69`, merge `b8c6002`.
- PR #4: head `c6cc5ba3eab20a9fc146c4f915298433ef7d77db`, merge `b77a3c2`.

Following explicit user approval, the existing feature branch was fast-forwarded
to `b77a3c2`. No new commit, push or PR was created. Maintaining and main were
not changed. Untracked editor directories were left alone.

`Core/Inc/gp_types.h` is the canonical revision-10 contract and is unchanged.
Public types, field layouts, enum values and function signatures are unchanged.
No separate revision-10 behavioral specification was present in the checkout,
so complete conformance to an external specification cannot be asserted.

## Verified defects and corrections

| Defect / root cause | Evidence and correction |
| --- | --- |
| Fractional steering reversal stalls | The reversal branch cleared the remainder on every tick. An original-source host reproducer at 1 pm/s retained steering `+1` after 1,000 x 1 ms reverse requests, where zero was expected. Removed that reset; both reversal directions now accumulate fractions and reach zero before moving the other way. |
| Command flags were ignored | Original source applied `{500,500}` with flags zero and returned `GP_OK`. Require VALID, reject reserved bits, and stop when ARM or DEADMAN is absent. All 256 flag combinations are tested. See the behavioral interpretation below. |
| NULL command/output retained mapping history | The argument-error path returned without clearing history. The original-source NULL-command reproducer retained `{500,500}`. Clear history and fractional accumulators on argument errors; zero the output wherever available. |
| Shared motor types redefined | Including `gp_types.h` with the old driver header produced compiler errors beginning with duplicate `GP_OK`. The driver now includes the canonical header; both include orders compile. |
| Stop left steering state and transitions alive | Original-source test reported `previous.steering_rate_pm == 500` after Stop. Stop now calls off, zeros both outputs, clears timestamp/direction state and preserves valid configuration/initialization. Repeated Stop is safe. |
| Setup wait occurred before signal establishment | The old code added DIR/ENA setup intervals to deadtime before the only write callback. That cannot enforce setup after DIR/ENA change. The driver now enforces only deadtime; the documented callback contract owns setup sequencing and returns `GP_PENDING` while inactive. |
| Adapter pending was treated as failure | Every non-OK write result previously invoked off and became `GP_IO_ERROR`, cancelling nonblocking setup. Pending now preserves adapter setup and returns zero applied/internal outputs. Actual failures invoke Stop and become `GP_IO_ERROR`. |
| Invalid driver configuration was not reliably initialized/off | Validation returned before off and left stale fields. Init now copies configuration before clearing state, invokes a provided off callback before numeric validation, and leaves failures uninitialized. A missing off callback cannot shut hardware down; Stop is required before reinitializing a live instance. |
| Impossible STEP pulse/period combination accepted | Independently nonzero frequency and pulse width could describe no low phase. Reject `max_step_hz * step_pulse_us >= 1,000,000` using 64-bit arithmetic. This is a generic feasibility check, not an approved motor limit. |
| Output aliasing destroyed inputs/history | Early output clearing erased the request when driver request/applied aliased, or mapper history when out referred to previous. Copy the driver request before output writes; retain mapper history until computation. Regression tests exercise both. |
| Libraries omitted from firmware CMake | Neither source was registered. A shared `gp_actuator` object-library target now supplies both implementations/includes to the firmware and host executable. |

## Explicit behavior and hardware boundary

Mapping uses per-mille caps (clamping, not rescaling) and independent axis slew.
Neutral stops each axis immediately. Reversal passes through zero in a separate
update. Disabled commands stop with `GP_OK` before numeric/timing validation.
Enabled commands with missing VALID, reserved bits, invalid ranges, zero dt or
dt above max return `GP_INVALID` and reset. Valid commands lacking ARM or DEADMAN
stop with `GP_OK`. This is a conservative interpretation of the supplied flag
definitions; Farah should reconcile these return semantics with any separate
revision-10 behavioral document.

Stop cancels pending work and does not retain a command to resume. A subsequent
fresh Apply with safety permission may start movement. Since Stop has no clock
argument, initial steering start and steering restart after Stop wait a full
deadtime from the first nonzero request. Ordinary steering-zero intervals keep
their stop timestamp; restarting steering waits any remaining deadtime.
Both motor outputs are inactive throughout driver deadtime and adapter pending.
A throttle-only command is stopped if steering requests a restart before its
deadtime has elapsed.

The required sequence is:

1. Driver calls off to stop STEP and throttle.
2. Driver waits `reversal_deadtime_ms` without blocking.
3. Driver invokes write; the adapter establishes DIR and ENA with STEP stopped.
4. Adapter returns `GP_PENDING` while enforcing DIR/ENA setup intervals.
5. A later write retry may activate outputs and return `GP_OK`.

The mock demonstrates this sequence, including that setup starts **after**
deadtime. It does not prove the electrical behavior of a real adapter.
While pending, the adapter must keep both outputs inactive, must not activate
outputs autonomously between calls, and must replace a pending request when a
new one arrives. Off must synchronously stop both motors and cancel pending
setup. Driver safety/error/neutral paths invoke off immediately.

The existing callback gets only per-mille output, with no separate prepare/start
operation or configuration argument. The adapter must obtain the same immutable
configuration through its user context and enforce `max_step_hz`, pulse width,
DIR/ENA polarity, and device-specific setup/minimum high/low intervals.
The core enforces throttle cap and steering per-mille range; it cannot measure
physical Hz or pin transitions. No physical adapter was supplied or implemented.

Clock handling uses unsigned subtraction across uint32 rollover. Callers must
use a monotonic wrapping millisecond clock and service transitions within
2^31 ms; configured deadtime must be below that bound. Calls must be serialized.

Farah's remaining integration decision is whether to adopt this documented
nonblocking adapter contract or introduce a future explicit prepare/start API.
The latter would require a coordinated shared-contract change. Approved motor
limits, physical timing, and emergency-stop hardware behavior still require
hardware review and measurement.

## Executed validation

- Baseline extracted directly from Maintaining: steering-reversal and Stop
  reproducers both exited 1 with the stale states above. The flag/NULL-history
  reproducer also exited 1. Combined-header compilation failed on redeclarations.
- Host configure: `cmake -S tests/host -B build/host -G Ninja` passed.
- Host build: `cmake --build build/host` passed with native MinGW GCC 8.1.0,
  C11, `-Wall -Wextra -Werror -Wpedantic`.
- `ctest --test-dir build/host --output-on-failure`: **18/18 passed**.
  The first run was 17/18 because the combined test exceeded its mock event-log
  capacity; increasing that test-only buffer resolved the failure.
- Both header include orders and both implementations compile and link in
  `gp_tests`; the combined test exercises the mapping-to-driver path.
- Existing firmware commands `cmake --preset Debug` and
  `cmake --build --preset Debug` passed with installed ARM GCC 14.3.1,
  CMake 4.3.1 and Ninja. Both new object files were compiled and passed to the
  firmware linker. No firmware compiler/linker failures were observed.
- CI specifies ARM GNU 14.2.Rel1, which was not the locally installed compiler;
  exact CI-toolchain validation remains outstanding. Remote CI was not run.
- Firmware has no actuator call sites yet, so section garbage collection may
  discard unused library functions; registration is not runtime integration.
- `git diff --check` passed. No motor activation or flashing occurred.

The 18 CTest cases are `map_endpoints`, `map_fractional`, `map_reversal`,
`map_flags`, `map_invalid`, `map_reset`, `map_config`, `map_large_dt`,
`driver_config`, `driver_stop`, `driver_safety`, `driver_limits`,
`driver_errors`, `driver_reversal`, `driver_pending`, `driver_rollover`,
`driver_alias`, and `combined`. Hardware tests use mock callbacks and synthetic
limits, including startup, repeated Stop, emergency-stop/disable, pending
cancellation/replacement, callback failures and wrapping timestamps.

## Files changed by this repair

- `CMakeLists.txt`
- `Core/Inc/command_map.h`
- `Core/Src/command_map.c`
- `lib/CMakeLists.txt` (new)
- `lib/motor_driver/include/motor_driver.h`
- `lib/motor_driver/src/motor_driver.c`
- `tests/host/CMakeLists.txt` (new)
- `tests/host/headers_reverse.c` (new)
- `tests/host/test_actuator.c` (new)
- `docs/BUILDING.md`
- `docs/ACTUATOR_INTEGRATION.md` (new)

The existing CI host job automatically discovers `tests/host/CMakeLists.txt`;
no workflow modification is required. Untracked `.vscode/` and `.settings/`
files are outside this repair and were not edited by the agent.
