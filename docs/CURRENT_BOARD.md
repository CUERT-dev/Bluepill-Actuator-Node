# Current-board firmware and unpowered signal validation

> [COMMISSIONING_CANDIDATE.md](COMMISSIONING_CANDIDATE.md) provides the generated
> real dual-axis commissioning HEX with provisional settings. Physical properties
> remain UNKNOWN; SignalTest remains available with motor energy disconnected.

This supersedes earlier instructions requiring five `GpApproved_*` providers.
The final board has no Bluepill-connected BTS7960 enables, EM882S ENA,
E-stop readback or interlock input. The current BSP represents that wiring
directly. It controls PWM, STEP and DIR; it cannot interrupt motor energy or
observe driver power/enable readiness.

## What builds

| Presets | CAN | PWM/STEP outputs | USART1 | IWDG |
|---|---|---|---|---|
| Debug, Release, SafeFlash | Off | Inhibited, analog GPIO | 115200 8N1 | Off |
| CanBench, CanBenchRelease | Real 500 kbit/s | Inhibited | Same | Off |
| SignalTest, SignalTestRelease | Real 500 kbit/s | Real TIM2/TIM3 adapter; software arming required | Same, with unpowered-test startup warning | Off |
| BenchActuation, BenchActuationRelease, Commissioning | Real 500 kbit/s | Real dual-axis actuation; software arming required | Same, with commissioning warning | Off |

All ten presets compile and **link** real firmware. SignalTest is a new active
waveform image for scope/logic-analyzer validation with motor energy physically
disconnected. This is an operator condition: the MCU cannot detect whether power
is disconnected. Selecting this profile does not make an energized mechanism safe.
Powered outputs require a separate controlled-bench mode and real commissioning
settings; the checked-in commissioning candidate is explicitly physically unverified.
SignalTest stays unpowered-only and rejects watchdog activation.

```text
cmake --preset SignalTest
cmake --build --preset SignalTest
cmake --preset SignalTestRelease
cmake --build --preset SignalTestRelease
```

The profiles use `App/Profiles/signal_test.h` without `GP_BOARD_SOURCES` or any
external board provider. Obsolete nonempty `GP_BOARD_SOURCES` configurations fail
with an explanatory message. The historical `gp_board_port.h` now contains only
a documentation pointer. No dummy callbacks replace its removed declarations.
The existing CAN/inhibited profiles retain their roles.

## Supervisor and output behavior

`GpControlInputs.estop_mode` distinguishes `GP_ESTOP_REQUIRED` (the default,
fail-closed behavior) from `GP_ESTOP_NOT_CONNECTED`. The current BSP explicitly
reports NOT_CONNECTED and leaves `estop_ok=false`. It never invents a healthy
E-stop/interlock signal. Unknown modes fail with a dependency fault. Required
readback configurations still fault when unhealthy, and an explicitly latched
E-stop fault still shuts down every mode.

`outputs_approved` now means software waveform/configuration permission: timer
initialization completed and an explicit signal-test or bench mode is selected. It does
not assert independent electrical safety. The existing `MotorSafety` object
passed to Boda carries the supervisor's effective output permission, not a new
physical readback.

The production protocol and supervisor remain in the path. A fresh valid neutral
ARM-low frame followed by a fresh ARM rising edge with DEADMAN is required.
STOP, ARM/DEADMAN loss, the 200 ms communication timeout, malformed/stale/reordered
frame rejection, bounded queues, urgent fault latches, generation barriers and
explicit recovery remain active. `recover` cannot arm or inject demand. Old
queued frames cannot cross shutdown/recovery. The prior slew/readiness correction
is preserved. Ordinary neutral stops only its axis; Boda's intentional both-output
pause during steering setup/reversal remains.

The real off path still disables TIM2_CH2 and TIM3_CH1, stops their counters,
clears compare values and drives the configured PWM/STEP pins inactive. It no
longer calls an imaginary physical inhibit. PB4 remains the inactive opposite
throttle input. STEP-off does not establish loss of holding torque; PWM-off does
not establish mechanical braking or independent energy isolation.

## Signal settings and steering timing

These values configure test waveforms, not approved powered motor operation:

| Parameter | SignalTest value |
|---|---|
| CAN | PB8 RX / PB9 TX, 500 kbit/s, ID 0x105, eight-byte production protocol |
| Throttle | PB3 L-PWM via TIM2_CH2 partial remap 1; PB4 R-PWM low |
| PWM | Active high, 20 kHz, cap 100 permille (10% duty) |
| Steering | PA6 STEP / TIM3_CH1, PA7 DIR; active-high STEP and positive DIR high |
| Steering scale | 1,000 Hz full-scale planner setting, 100-permille cap: at most 100 Hz output |
| Slew | 1,000 permille/second for each axis |
| STEP high / low minimum | 5 us / 5 us |
| DIR setup | At least 5 us in the adapter before STEP starts |
| Reversal deadtime | 10 ms in Boda, then adapter DIR setup |
| ENA setup field | Zero: no ENA control or enable transition is implemented |

PB3/L-PWM naming and positive DIR polarity do not confirm mechanical direction.
Pins PA9/PA10 remain USART1; PA13/PA14 remain SWD. TIM3 is not remapped to PB4.

The hardware adapter continues to enforce STEP high/low timing, DIR setup,
direction changes, pending progression and STOP cancellation. Its optional
`prepare` hook remains available to an actual future electrical-readiness
implementation and its existing long-readiness tests remain. The current BSP
supplies NULL because it has no such mechanism. NULL means readiness is
**unimplemented/unverified**, not successful. No unconditional-success provider
or invented 200 ms software delay was added.

An actual EM882S enable/power transition has electrical settling requirements
beyond DIR-to-STEP setup. SignalTest cannot prove those requirements because ENA
and power-ready feedback are absent. Powered commissioning must establish the
installed driver's stable-power/enable conditions and signal timing at its
terminals; a software wait alone cannot establish them. The previously reviewed
manufacturer references remain in `MAINTAINER_BRINGUP.md`.

## Physical checks still required

No image was flashed and no physical tests were performed during this refactor.
With motor energy disconnected, an operator must verify boot, UART, real CAN
reception, pin levels and the PWM/STEP/DIR waveforms, including first/last pulses,
reversals, independent neutral, STOP, timeout and recovery. Stream incrementing
commands faster than the 200 ms timeout after the neutral/ARM handshake. The UART
service still supports only `status` and `recover`.

Powered operation additionally needs reviewed machine limits, electrical levels,
direction, driver current/microstep configuration, power/enable settling, reset
behavior and an appropriate physical power-interruption procedure/interface.
Those are commissioning and system-safety questions, not missing current-board
GPIO functions. No claim of powered readiness follows from this refactor.

## Future hardware revision

Independent E-stop, interlock and controllable enable/readback integration are
hardware-revision considerations. Design actual cutoff paths and default states,
identify their MCU pins and polarities, define permission versus driver-ready
feedback, and validate enable/power transition timing before adding a provider.
If a readback is added, select REQUIRED mode rather than treating absence as
healthy. Keep callbacks bounded and cancellable and retain independent cutoff
effectiveness during CPU failure. There is no requirement to invent these five
interfaces for the present wiring or for unpowered waveform testing.

## Verification for this change

- Full native suite: **122/122 passed**, including all original 116 cases.
  Four new header gate cases and two new supervisor cases cover the unpowered
  profile, rejection of powered configurations, honest absent-input handling,
  unknown-input modes and fault/recovery behavior. The full pipeline fixtures
  now explicitly model absent E-stop wiring with `estop_ok=false`.
- Optimized native Release: **122/122 passed**.
- MSVC AddressSanitizer: **108 runtime cases passed**; the other 14 CTest cases
  are compiler configuration gates. Native GCC lacks ASan/UBSan runtime libraries;
  UBSan and cppcheck remain unavailable locally.
- All **seven ARM images linked**, including both real active signal adapters.
  Per-image ELF checks verify kernel/control/codec symbols, strong CAN callbacks,
  exception/CAN/UART vectors, profile-specific active/inhibited symbols and the
  absence of every `GpApproved_*` symbol. SignalTest links real HAL timer init and
  configuration; its watchdog init symbol is absent.
- Seven negative CMake configurations pass their expected rejections, including
  unsupported powered operation, incomplete signal mode, watchdog conflicts,
  obsolete board sources and test-fixture approval headers.
- Strict enabled-branch compilation and GCC `-fanalyzer` on eleven translation
  units pass. Peripheral ownership and staged/unstaged whitespace checks pass.
- UART/CAN/MCU/timer electrical behavior remains physically unverified.

Exact commands/logs are under ignored `build/current-board/`, notably
`run-checks.py`, `checks.log`, `host-release.log`, `asan/build-run.log`,
`analyzer.log` and `configuration/*.log`. Each `build/<preset>/` contains fresh
ELF, HEX, MAP, size and SHA manifests. Manifests explicitly record unpowered
signal mode, `powered_operation_approved=false` and absent physical safety inputs.
The original index and existing work are preserved; no flashing, staging,
commits, pushes, merges or changes to other repositories were performed.
