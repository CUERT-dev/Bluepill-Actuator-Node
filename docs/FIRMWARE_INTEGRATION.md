> **21 October maintainer update:** See [MAINTAINER_BRINGUP.md](MAINTAINER_BRINGUP.md)
> and [MAINTAINER_VALIDATION.md](MAINTAINER_VALIDATION.md). CAN 500 kbit/s bench
> profiles, UART diagnostics/recovery and inhibited-supervisor operation now exist.
> The earlier status below is retained as historical context. Motors and IWDG
> remain disabled pending real hardware prerequisites.

**Firmware integration, 20 October 2026**

The project now builds a real FreeRTOS application with the existing portable
CAN/safety supervisor and repaired mapping/driver libraries. The supplied
production revision-10 CAN codec is now built in, and ordinary neutral
transitions stop only the affected axis. The default image
is inhibited: CAN startup, motor actuation and IWDG activation are OFF. Their
unapproved values remain explicit placeholders. No board has been flashed or
motor powered by this work.

The final specification is preserved in
[CUERT_Bluepill_Final_Integration_Brief.md](CUERT_Bluepill_Final_Integration_Brief.md).
The later [final schematic](hardware/final_pinout.png) replaces the brief's
provisional wiring. Physical pin assignment is now confirmed; direction, timing,
limits, polarities, enable/E-stop and reset-safety behavior remain unapproved.
The [focused schematic review](FINAL_SCHEMATIC_REVIEW.md) records the exact
GPIO/remap audit and separates completed software from configuration and physical
work. Operational CAN and powered operation remain gated.

**Implemented application wiring.** `App/Src/gp_app.c` creates static motor and
CAN tasks at application priorities 3 and 2, with 512-word and 384-word stacks.
The idle task also uses static allocation. There is no RTOS heap allocation,
STOP task, diagnostics task or logging in interrupts. The actual FreeRTOS 10.3.1
Cortex-M3 kernel/port comes from the installed STM32Cube F1 V1.8.7 package;
the added HAL CAN/TIM/IWDG files come from that same package. All 36 imported
files and their exact hashes are recorded in
[vendor-manifest.json](vendor-manifest.json), with upstream licenses retained.

Startup initializes the board infrastructure, creates tasks and starts the
scheduler. The motor task requests off before configuring the driver or enabling
CAN interrupts. It owns normal output writes, mapping/driver initialization,
supervisor runs, local recovery and watchdog feeding. Early board initialization
only establishes the reset/inhibited hardware state before tasks exist.

The task calls `GpMotor_Run` on each notification and each nominal 10 ms
deadline. Absolute deadlines survive repeated notifications; there is no busy
catch-up loop. Real elapsed time, timeout, ordered safety events, generation
barriers and `GP_PENDING` behavior remain intact. Only the ordinary-neutral
policy in the foundation changed in this follow-up. When
motor limits are missing, the controller is not initialized with invented
limits: the motor task remains in an explicit off-only mode and reports that
blocker. Thus the default image runs the kernel/tasks, but does not run an
operational command-to-motor path.

The supervisor now preserves ordered neutral transitions independently per axis.
Steering neutral stops STEP movement while valid throttle continues; throttle
neutral stops throttle while ongoing steering continues. Both neutral produces
zero movement on both axes and remains ARMED when VALID/ARM/DEADMAN stay set.
A per-run axis mask prevents coalescing from hiding an intermediate zero, even
when a later command in the same batch is nonzero. That axis can resume on a
later scheduled update. Only its slew history/remainder is cleared. A neutral
notification before the next period commits the stop immediately, including at
zero elapsed time, without advancing the other axis's slew or shifting its
mapping deadline. No synthetic elapsed time is supplied to the mapper.

STOP (VALID low), permission loss, timeout, E-stop, bus/queue/timing faults and
driver errors still use the original `MotorDriver_Stop` shutdown and generation
barrier, clearing both stages and requiring the existing recovery/arming rules.
The canonical API, `command_map.c` and `motor_driver.c` are unchanged.

Boda's callback contract still requires **both outputs inactive** during steering
startup/reversal deadtime or adapter DIR/ENA setup (`GP_PENDING`). Restarting
steering too soon after neutral can therefore briefly interrupt throttle. This
is a driver/adapter contract limitation, not a neutral-triggered safety shutdown;
no incompatible independent-output workaround was introduced.

SVC and PendSV use naked forwarding branches into the real FreeRTOS port;
SysTick advances HAL time and, after scheduler startup, the kernel tick. The
1 kHz bridge is checked at compile time. CAN interrupt priority 6 is checked
against the kernel syscall threshold 5 and four implemented NVIC priority bits.
Channel critical sections save/restore PRIMASK with memory barriers. They contain
no blocking RTOS calls; notifications occur after leaving them. The independent
physical E-stop must remain effective regardless of interrupt masking.

**CAN reception and decoder boundary.** The BSP contains real HAL initialization,
PB8/PB9 final route/remap, standard data-frame filter for `0x105`, FIFO/error notifications,
and RX0/RX1/SCE interrupt connections. Startup requires explicit CAN approval and
prescaler/BS1/BS2/SJW/bitrate values. The bitrate equation is checked against the
actual APB1 clock; there is no default operational bitrate.

RX copies at most three FIFO0 frames per callback, preserving IDE/RTR metadata,
and calls the existing ISR capture interface, which takes the local reception
timestamp. FIFO overrun is latched before HAL clears its flag. Bus-off is latched
and wakes motor urgently. A full raw queue also masks RX notifications to avoid
an interrupt storm starving shutdown. Recovery is explicit, from the motor task
after outputs are off; it may reset/reinitialize CAN and then invoke the
supervisor recovery barrier. There is no automatic bus-off rearm.

The shutdown hook drains at most three hardware entries per FIFO. Remaining
data or unacknowledged overrun causes the barrier to fail closed. Software
generation tags still reject an old decode completing after shutdown. HAL
initialization and manual CAN recovery may use HAL's bounded transition waits;
normal motor callbacks and ISR processing do not wait for time to pass.

The supplied `GP_DecodeCan` and `GP_EncodeCan` are integrated unchanged in
`lib/can/src/gp_can_codec.c`, using only canonical `Core/Inc/gp_types.h`
(API revision 10). They validate command ID `0x105`, DLC exactly 8, little-endian
16-bit sequence/throttle/signed steering, and CRC-8/SMBUS over bytes 0..6
(polynomial `0x07`, initial value 0, no reflection, final XOR 0). Byte 6 carries
flags and byte 7 carries CRC. Throttle is 0..1000, steering -1000..1000, and only
VALID/ARM/DEADMAN are permitted. VALID-low is structurally accepted only when
both axes and all flags are zero; this successful decode reaches the supervisor's
urgent STOP latch. Decode/encode errors preserve the caller's output.

`gp_can_codec` is an object target shared by host tests and firmware. The app
passes the real `GP_DecodeCan` directly to `GpCanRx_Init`; there is no absent
provider or fallback branch. Both public codec functions are retained in the
linked ELF; the encoder does not imply CAN transmission from this receive-only
application. Obsolete external `GP_DECODER_SOURCES`/`GP_DECODER_INCLUDE_DIRS`
settings are rejected to prevent duplicate providers. Both images pass a check
for exactly one strong symbol for each codec entry point. There is a single
`gp_types.h`; no public type definition was copied into the codec.

Malformed/CRC/range/flag failures, extended/remote frames, and replayed/out-of-order
sequences never publish a command or renew freshness. Real-decoder integration
tests verify eventual timeout under rejected traffic, sequence rollover,
STOP despite a full event queue, and recovery-generation flushing. The original
raw/event queues, ISR connections, fault handling and recovery barriers are
preserved. Compatibility with the actual sender/gateway still needs a bus
capture; a sender's lost transient STOP cannot be inferred from sequence gaps.

**PWM, STEP and shutdown callbacks.** `lib/hardware` supplies the adapter policy
used by the real BSP and host tests. It computes 16-bit prescaler/ARR/CCR plans,
checks pulse high/low feasibility and caps, stops both waveforms before a
direction change, establishes DIR/any approved ENA state, and waits the configured
setup intervals through `GP_PENDING`. Slew changes during setup do not restart
the setup timer. Off cancels pending state, and elapsed time alone never starts
an output. Neutral never releases an external inhibit.

The actual STM32 backend uses TIM2 channel 2 and TIM3 channel 1. No task loop
toggles STEP. Timer preloads update running rates at a natural timer boundary;
startup connects an inactive reference before starting the first pulse. Off
clears channel enable/CEN/CCR and, only in an approved actuation profile,
establishes the specified GPIO inactive states and invokes the approved inhibit
port. The DWT cycle counter is extended into a wrapping microsecond clock with
fractional accumulation; it is serviced by the motor task and must not go an
entire cycle-counter wrap without service (about 59 seconds at configured
72 MHz).

Frequency and pulse widths are quantized to timer ticks. STEP frequency uses
ceiling quantization to whole Hz within the configured maximum; timer division
then avoids exceeding that frequency. Pulse high/low durations round
conservatively to representable ticks. These are arithmetic checks, not measured
electrical timings. Motor task retries can make an approved setup wait longer
than its microsecond minimum.

The final physical wiring is below; electrical operation remains gated:

| Function | Final connection | Default behavior / blocker |
| --- | --- | --- |
| CAN | PB8 RX / PB9 TX, bxCAN remap | Not initialized without approved pin map/timing; real decoder is linked |
| STEP | PA6 / TIM3_CH1 default map | Timer output disabled; pin analog |
| DIR | PA7 GPIO | Pin analog; polarity unknown |
| BTS7960 L-PWM | PB3 / TIM2_CH2 partial remap 1 | Timer output disabled; forward input unconfirmed |
| BTS7960 R-PWM | PB4 GPIO | Pin analog; inactive level must be approved separately |
| USART1 | PA9 TX / PA10 RX, default map | Unused; no baud/protocol or application transport specified |
| Debug | PA13/PA14 SWD | SWD retained, JTAG disabled; no SWO on PB3 |

`GP_THROTTLE_FORWARD_ROUTE=0` means forward is unselected. Route 1 is PB3/L-PWM.
Final wiring does not establish whether this input drives the motor forward.
Selecting PB4/R-PWM is rejected: an approved timer/pin reassignment is required,
and firmware never moves TIM3_CH1 away from PA6 silently.

The final schematic shows BTS7960 enables tied high and EM882S ENA unconnected;
it supplies no MCU-controlled enable, E-stop readback or independent energy
isolation. Firmware cannot implement the missing circuit through callbacks.
Analog/high-impedance pins and disabled timers are software inhibition, **not
proof that an externally powered driver cannot energize**. Approved external
pulls/default-off circuitry and a physical E-stop are still required.
With EM882S ENA unconnected, pulse-off can leave holding torque. Zero steering
does not center the wheels. No software Stop is claimed to physically brake the
DC motor.

**Approval and board-port interfaces.** `App/Inc/gp_board_config.h` records the confirmed physical pin map with
`GP_PIN_MAP_APPROVED=1`; all electrical settings retain zero or explicitly
unknown placeholders. Future reviewed values are supplied through
`GP_BOARD_APPROVAL_HEADER`. CMake hardware options default OFF and require a
real approval header; motor activation also requires approved CAN startup and
`GP_BOARD_SOURCES`. Test fixtures and test provider sources are rejected as
production configuration. Timing overflow/truncation, unsupported routes and
unknown polarity fail compilation or initialization rather than becoming
operating defaults.

The declarations and required contract are in `App/Inc/gp_board_port.h`.
The approved board source must implement these real, bounded functions before
an actuation profile can link:

- `GpApproved_EstopHealthy` and `GpApproved_InterlockHealthy`: actual circuit
  readback, never unconditional healthy placeholders.
- `GpApproved_Inhibit`: establish the approved inhibit state synchronously and
  idempotently. Its physical effect must be verified.
- `GpApproved_ReleaseInhibit`: release the approved hardware permission only
  under its reviewed conditions.
- `GpApproved_PrepareSteering`: establish any actual ENA/setup signals before
  the configured setup wait begins. If ENA is not wired, that absence and the
  pulse-off/holding behavior need explicit approval; firmware cannot synthesize
  an enable cutoff or claim an ENA timing guarantee.

These functions have no supplied fake implementation. DIR/ENA/inhibit ordering
and any power-stage startup delay must be reviewed with the actual circuit;
new delays cannot be hidden behind a boolean callback or an asynchronous start.
The existing driver callback contract still prohibits autonomous activation
between `GP_PENDING` retries.

`GpApp_RequestRecovery` is a task-context local acknowledgement interface.
No physical recovery pin or CAN recovery command is invented. The motor task
processes it only in a fault state, requests off, optionally restarts CAN and
asks the supervisor to recover. New post-barrier neutral/ARM-low and ARM-edge
commands are still required.

**Watchdog and fatal faults.** Real HAL IWDG support and dual-task progress
supervision are implemented, with startup disabled by default. Starting IWDG
requires reset-safe hardware and approved watchdog settings, including the
prescaler, reload, task deadline and maximum LSI frequency. The shortest
calculated watchdog interval must exceed the supervision deadline plus a motor
period. LSI tolerances and actual reset latency still require measurement.
Motor refreshes IWDG only while both motor and CAN workers have progressed;
notifications alone cannot fake worker progress. The CAN task has a bounded
periodic health wake even without traffic.

Reset flags and startup blockers are available in `gp_boot_status`. Stack
overflow/assert/CPU faults record a panic reason and stop normal execution;
reset is requested only if reset-safe hardware was approved. The optional HAL
`assert_failed` hook now reaches this same fatal path; it no longer returns
without handling a failed HAL assertion. A damaged kernel
cannot safely invoke normal RTOS/motor callbacks. Independent hardware inhibition
and the commissioned watchdog/reset behavior must cover that failure class.
IWDG startup failure in a requested watchdog profile inhibits permission.

**CubeMX and regeneration.** Serial Wire was selected in the actual `.ioc`.
The installed CubeMX quiet CLI generated the SWD MSP code into an isolated
directory, and that genuine MSP output was integrated. Generation was then
repeated using the application's actual USER CODE sections; startup calls,
SVC/PendSV forwarding, SysTick and RX0/RX1/SCE hooks survived. The checker is:

```text
python scripts/check_cubemx_hooks.py --src build/cubemx-check/Src
```

The quiet-mode invocation follows the official
[CubeMX CLI reference](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_CLI.html).

Peripheral/application registrations live in `cmake/firmware/CMakeLists.txt`,
outside the CubeMX-owned CMake subdirectory. Application logic and native RTOS
configuration live in `App`. The full operational CAN/PWM/STEP `.ioc` cannot be
generated responsibly before operating timing, polarity and forward-route approval; the real HAL
initialization backend is already written and gated. Do not add a second
CubeMX/CMSIS RTOS startup path alongside the native kernel. When an approved
peripheral `.ioc` is adopted, reconcile newly generated CAN handlers/init calls
with this BSP and rerun the hook and vector checks; do not silently initialize
outputs before the approval gates.

**Executed validation and artifacts.** The final host run passed **102/102**
(0 failed): all 79 prior cases plus 7 codec cases (including the exact supplied
test), 6 real-decoder/supervisor cases, 9 neutral/safety regressions and 1 real
driver/adapter-policy neutral case. The codec tests cover supplied golden motion
and STOP vectors, endian layout, every single-bit packet corruption, exhaustive
16-bit throttle/steering decode ranges, all flag bytes, all DLC/ID alternatives,
null arguments and output preservation. The independent test CRC uses polynomial
long division rather than the production helper.

Neutral regressions cover each axis at and before its period, zero elapsed time,
coalesced zeros, both neutral, fractional slew preservation, pending steering
cancellation, driver failure and a fault arriving before final commit. All
previous STOP/timeout/E-stop/recovery tests still pass. Test hardware callbacks
are mocks, not electrical validation; the production codec and supervisor used
by the integration cases are real.

Debug and Release cross-builds passed with installed ARM GCC 14.3.1, CMake 4.3.1
and Ninja. The powered command path remains gated by hardware approvals. Both
binaries passed checks for real kernel/control symbols, strong CAN callbacks, correct exception/CAN
vectors and stack-preserving SVC/PendSV branches. The default binaries contain
the real encoder/decoder and no
`HAL_TIM_PWM_Start`, active hardware write policy, or `HAL_IWDG_Init`.
Enabled CAN/actuator/watchdog branches were separately type-checked with
`-Wall -Wextra -Werror -Wpedantic` and synthetic compile-only settings; no enabled
firmware image or mocked firmware decoder was linked.

| Image | Flash used / 65,536 bytes | Linker RAM reserved / 20,480 bytes |
| --- | ---: | ---: |
| Debug | 21,808 | 7,336 |
| Release | 13,108 | 7,192 |

RAM includes static task stacks and linker reservations; it is not a runtime
stack high-water measurement. Stack sizes and worst-case critical-section/ISR
execution time still require target measurement with the production decoder.
The exact CI compiler 14.2.Rel1 and remote CI were not run.

Each `build/Debug` and `build/Release` directory contains:

- `Bluepill-Actuator-Node.elf`, `.hex`, and `.map`;
- `firmware.size.txt`, `SHA.txt`, and `firmware-manifest.json`;
- the manifest's actual HEAD, dirty-tree source fingerprint, artifact SHA-256
  values, compiler/build type, feature gates and linker memory footprint.

The recorded HEAD is `b77a3c24822c4722dc4f9ab325edbb86ea09073c`; all new integration
work remains uncommitted, so HEAD alone does not describe the built image. Both
images record the same source fingerprint. The feature manifest now reports
`decoder=true`, with `can=false`, `actuation=false` and `watchdog=false`. Every
normal build refreshes the artifact manifest. The review bundle is `build/firmware-artifacts.zip` with its
own `.sha256` file.

Executed commands include the host configure/build/CTest sequence, both firmware
presets, `gp_check_enabled_syntax`, the linked-image checks and CubeMX hook
checker. ARM GCC `-fanalyzer -Wall -Wextra -Werror -Wpedantic` passed for the codec,
supervisor, mapper, driver and hardware policy with no diagnostics. `cppcheck`
is unavailable on the local PATH; remote CI was not executed.

Four isolated CMake configurations correctly rejected missing hardware approval,
a duplicate decoder provider, a test fixture used as an approval header, and
missing board-port sources. A fifth configuration found the built-in codec with
CAN enabled and an explicitly unapproved, empty header; this was configuration
only, never an enabled build or hardware approval. The 7 existing preprocessor
configuration tests also pass. All 36 vendor hashes still match. `git diff --check`
passed. No flash, commit, push or merge was performed.

**Remaining decisions before powered operation.**

- The supplied codec dependency is resolved, including well-formed invalid-neutral
  STOP. Confirm real sender/gateway compatibility and transient ADC-invalid
  behavior: a single lost upstream STOP followed by valid sequences cannot be
  inferred safely from a sequence gap. No protocol workaround was invented.
- Physical pin assignment is final. Approve PB3/L-PWM forward behavior or a
  reviewed PB4-compatible solution, CAN bit timing/transceiver, PWM frequency/cap
  and slew limits.
- Approve STEP high/low minima, DIR/ENA setup and reversal timing, maximum
  frequency, electrical polarities and inactive GPIO levels.
- Finalize independent physical E-stop, driver enables/inhibit/readback,
  reset-default states, throttle power/logic supply and holding-torque policy.
- Supply the real board-port functions, local recovery source and approved
  watchdog/reset strategy. Generate/review the operational peripheral `.ioc`.

Commissioning must verify, with motors initially disconnected, startup/reset
inactive levels; SWD/remap ownership; PWM/STEP/DIR/ENA waveforms and quantization;
all caps/setup/deadtime; STOP and overflow/bus-off/timeout/E-stop cutoff latency;
no pre-shutdown rearm; pending cancellation; task stack high-water marks and
worst-case bus/ISR/task load; watchdog task-stall and reset behavior; and the
independent E-stop's physical effectiveness. These checks have **not** been
performed, and this handoff is not hardware commissioning or vehicle-safety
approval.

There is no remaining missing decoder implementation or known failing code test.
The hardware-dependent board-port implementations, local recovery source and
operational peripheral `.ioc` still need the actual circuit decisions above;
they are remaining integration work, not supplied fake hardware functionality.
Steering deadtime/setup coupling is retained as a documented contract limitation.

The exact inventory for this codec/neutral follow-up is in
[CODEC_NEUTRAL_CHANGED_FILES.txt](CODEC_NEUTRAL_CHANGED_FILES.txt). The earlier
firmware-step inventory remains in
[FIRMWARE_CHANGED_FILES.txt](FIRMWARE_CHANGED_FILES.txt). Earlier staged repairs,
shared revision-10 types, mapper/driver implementations and all original cases
were preserved. The supervisor's neutral policy and its test harness changed
intentionally. New integration changes remain unstaged.

The latest focused step and its revalidation are recorded in
[FINAL_SCHEMATIC_REVIEW.md](FINAL_SCHEMATIC_REVIEW.md), with exact changes in
[SCHEMATIC_REVIEW_CHANGED_FILES.txt](SCHEMATIC_REVIEW_CHANGED_FILES.txt).
