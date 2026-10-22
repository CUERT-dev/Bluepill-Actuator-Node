# Maintainer firmware and bring-up — 21 October 2026

> Current-board profiles and safety-interface policy have changed. Use
> [CURRENT_BOARD.md](CURRENT_BOARD.md). The five providers described below are
> historical proposals, not requirements for current-board signal validation.

This supersedes the earlier handoff's absent UART, unselected recovery source,
and unapproved CAN timing statements. The final schematic remains
[final_pinout.png](hardware/final_pinout.png). No board, CAN bus or motor was
connected during implementation. All target/electrical measurements below are
instructions, not completed results. The PR target is **Maintaining**.

## Profiles and ownership

| Profile/preset | CAN | UART service | Motors | IWDG |
|---|---|---|---|---|
| `Debug`, `Release`, `SafeFlash` (A) | Off | 115200, 8N1 | Inhibited | Off |
| `CanBench`, `CanBenchRelease` (B) | Real bxCAN, 500 kbit/s | Same | Inhibited | Off |
| Approved custom build (C) | Required | Same | Blocked pending actual board port and settings | Separately gated |

Both B builds link the production codec and real HAL initialization, filters,
interrupts and recovery. They are ready for **unpowered-motor CAN bench testing**,
subject to checking the MCU/transceiver supplies and logic levels first. A/B do
not initialize a motor driver with invented numerical limits. Instead the motor
task runs `GpMotor_InitInhibited`/`GpMotor_Run`: the real fault supervisor and
generation barrier execute with a mandatory dependency fault and real off hook.
There is no fake healthy E-stop, output permission or simulated motor adapter.

This distinction is deliberate: in A/B, structural decoding is observable, but
the control channel remains barred, `publish=0`, `requested=0,0`,
`applied=0,0`, and state is `GP_FAULT_SHUTDOWN`. `recover` can restart CAN in B;
it cannot clear the missing motor/safety prerequisites or arm. Timeout, stale
sequence rejection, fresh ARM-edge recovery and applied motor behavior are
host-tested in the real control implementation. Their **armed target tests**
require C. A/B must never be described as a successful powered command path.

The existing `.ioc` owns the 8 MHz HSE/72 MHz SYSCLK/36 MHz APB1 clock tree and
SWD startup. The BSP exclusively owns gated CAN and motor initialization; the
service owns USART1 registers. Native FreeRTOS is the sole scheduler. This is
an operational hand-written peripheral backend outside generated CMake files;
adding unconditional `MX_CAN_Init`, timer, UART, IWDG or CMSIS-RTOS startup calls
would bypass these profiles. `check_peripheral_ownership.py` rejects them.
There is no remaining need to generate a second initialization implementation.
If ownership is later transferred to CubeMX, transfer the gates and USER hooks
in the same change and repeat regeneration/vector tests.

## Established settings and component bounds

CAN uses PB8 RX/PB9 TX, remap case 2, standard data ID `0x105`, DLC 8.
At APB1 = 36 MHz, prescaler 4, BS1 15, BS2 2, SJW 1 gives
`36,000,000 / [4*(1+15+2)] = 500,000 bit/s`; sample point is 88.889%.
HAL segment encodings are verified as `BTR=0x001e0003` in normal mode.
The runtime check rejects a different bitrate equation. CAN IRQ priority 6 is
legal for the FreeRTOS syscall threshold 5. No auto bus-off rearm is enabled.
`App/Profiles/can_500k.h` can also be included by the eventual motor header;
`can_bench.h` additionally forbids motor/watchdog activation.

Final routes remain TIM2_CH2 partial-remap-1/PB3 L-PWM, PB4 R-PWM held at its
approved inactive level, TIM3_CH1/PA6 STEP, PA7 DIR, USART1 PA9/10, SWD PA13/14.
JTAG/SWO must remain off. PB3 L-PWM **does not establish forward direction**.
If R-PWM proves forward, the smallest wiring change is swapping the two PWM
input connections (or a separately reviewed reversal of motor leads, if the
machine permits it). To preserve PB4 as the PWM output, use TIM3 partial remap
CH1/PB4 and move STEP to an independently routable timer pin, e.g. TIM2_CH2/PB3
after rerouting the driver inputs. PA6 cannot simultaneously remain TIM3_CH1
STEP under that remap. A fixed-wiring alternative would require a separately
designed DMA-to-GPIO waveform backend; it is not implemented or equivalent to
the existing timer-channel adapter. No route was silently changed.

Manufacturer limits are enforced at motor-profile compilation:

| Setting | Documented constraint implemented | Still needs a machine decision |
|---|---|---|
| BTS7960 PWM | At most 25 kHz | Frequency, duty cap, slew and module thermal/current limits |
| EM882S STEP | At most 200 kHz; high and low at least 3 integer microseconds (rounding 2.5 upward) | Actual cap, microstep/current settings and level-shifter performance |
| DIR setup | At least 5 microseconds | Sign/polarity, reversal deadtime and mechanical limit |
| Enable/power readiness | Before DIR; provider can return `GP_PENDING` for long waits | Actual wired enable/readback and power stabilization |

Sources: [Infineon BTS7960 datasheet](https://www.infineon.com/dgdl/BTS7960_Datasheet.pdf?fileId=db3a304412b407950112b43945006d5d&folderId=db3a304412b407950112b408e8c90004)
and [Leadshine EM882S manual](https://www.leadshine.com/upfiles/downloads/df11830f96faa1fe4df379db040bb17f_1650880565511.pdf).
The latter specifies 4.5–5 V high, 0–0.5 V low, ENA before DIR by at least 5 us
and enable time at least 200 ms. Its sequence chart requires ENA ahead of DIR by
at least 200 ms (stricter than the connector table's 5 us statement); require
that longer interval before DIR. Confirm the installed revision and settings.
The driver can remain enabled with ENA unconnected. These device limits do not
approve the assembled module, electrical levels or vehicle operating envelope.

The hardware adapter now has a cancellable readiness phase before DIR changes.
It retries the real provider without busy waits and keeps both waveforms off.
Only after readiness does the existing DIR/setup interval begin. Long enable
settling must not be squeezed into `MotorDriverConfig.enable_setup_us` (16 bits).
The new board-port `GpApproved_PrepareSteering` returns `GpResult`: `GP_PENDING`
while establishing real readiness, `GP_OK` only when verified, otherwise failure.
No shared revision-10 type or mapper/Boda driver contract was changed.

Timer backend audit: PWM mode 1, correct CCx polarity, ARR/CCR preloads, buffered
PSC, channels disabled at initialization, inactive GPIO latch before AF connect.
Running writes set UDIS while writing PSC/ARR/CCR so one overflow cannot latch
a mixed plan; the next natural update transfers the complete plan. No running
UG truncates a pulse. STEP starts with CNT at ARR, its reference inactive before
AF connect, then CEN. PWM at 100% is a continuous active level; it need not have
an inactive first tick. Off disables channel/counter and restores approved GPIO
levels. These are register/code findings; scope validation is still required.

## Blocked board functions and minimum interfaces

| Function | Why present schematic cannot implement it | Required real mechanism |
|---|---|---|
| `GpApproved_EstopHealthy` | No independent E-stop/readback shown | Independent energy cutoff plus fail-safe MCU status input; wire/polarity assignment reviewed |
| `GpApproved_InterlockHealthy` | No interlock/driver-ready readback | Actual permission/readiness feedback; distinguish safe-to-enable from already-enabled to avoid circular gating |
| `GpApproved_Inhibit` | BTS enables tied high; ENA unconnected | Synchronous controllable inhibit with verified physical effect and default-off circuitry; independent E-stop remains effective on CPU failure |
| `GpApproved_ReleaseInhibit` | No controllable permission output | Reviewed permission output; bounded and idempotent, never restarts settling on each retry |
| `GpApproved_PrepareSteering` | No ENA or power-ready interface | Wired enable and timestamp/readback, or reviewed external stable-enable/power-ready mechanism; enforce long settling and cancel on Inhibit |

All five remain intentionally **unimplemented**. No motor-enabled image is
provided. Once the circuit exists, implement these functions in `GP_BOARD_SOURCES`
including `gp_board_port.h`. Do not replace them with constant healthy/no-op
functions. Independent E-stop and reset-default hardware must be verified before
setting their approval flags. Enabled waveform control, setup/reversal, mapping,
CAN tasks, diagnostics and recovery are already connected in the application.

PWM/STEP-off does not disconnect energy. With tied-high enables, the two BTS
half-bridge outputs can produce electrical braking depending on the actual module
truth table; software does not guarantee coasting, braking or a safe energy state.
Unwired ENA can retain holding torque. Steering zero is not wheel centering.
Analog GPIO in A/B is high impedance, not a guaranteed driver inactive level.
PB4 has a debug/reset pull-up; check external pulls and all reset/power sequences.

For C, supply the actual remaining configuration in a reviewed header:
forward route; PWM frequency/cap/slew; steering cap/slew/max STEP rate;
STEP high/low; DIR/enable setup and reversal deadtime; PWM/STEP/DIR/opposite-input
polarities; motor, independent-E-stop and reset-safe approvals. The header may
include `can_500k.h`. Existing zero/-1 values remain placeholders. Choose limits
within the documented and timer-representable constraints, then measure them.
IWDG additionally needs prescaler/reload, task deadline, maximum LSI frequency
and reset-safe approval. Validate shortest watchdog timeout exceeds the task
deadline plus 10 ms; validate longest timeout against the system cutoff need.
No watchdog starts in A/B. Check hardware-start watchdog option bytes separately.

## Build and inspect before flashing

Install CMake >=3.22, Ninja, Python3, a native C compiler for tests and ARM GNU
tools on PATH. Local validation used CMake 4.3.1/ARM GCC 14.3.1/native GCC 8.1.
On Windows add the native compiler's bin directory to PATH as well as specifying
its executable; its helper DLLs otherwise may not load.

```text
cmake -S tests/host -B build/host -G Ninja
cmake --build build/host
ctest --test-dir build/host --output-on-failure
cmake --preset SafeFlash
cmake --build --preset SafeFlash
cmake --preset CanBench
cmake --build --preset CanBench
cmake --preset CanBenchRelease
cmake --build --preset CanBenchRelease
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
cmake --build build/Debug --target gp_check_enabled_syntax
python tests/firmware/test_configuration.py
python scripts/check_peripheral_ownership.py
```

Each profile directory contains ELF, HEX, MAP, size, SHA and a manifest of real
feature gates/source fingerprint/toolchain/memory/artifact hashes. Confirm
actuation/watchdog false in A/B and CAN true only in B. The enabled syntax target
uses synthetic values and does **not** create an operational motor image.
After real C prerequisites exist, configure a separate directory:

```text
cmake -S . -B build/MotorApproved -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -DGP_ENABLE_CAN=ON -DGP_ENABLE_ACTUATION=ON -DGP_ENABLE_WATCHDOG=OFF -DGP_BOARD_APPROVAL_HEADER=/absolute/reviewed_board.h -DGP_BOARD_SOURCES=/absolute/reviewed_board.c
cmake --build build/MotorApproved
```

The paths above are placeholders, not supplied providers. Compile/configuration
success is not proof that manually asserted hardware approval flags are true.

## Flash and bench sequence (operator actions, not executed here)

1. Equipment: ST-Link with SWDIO/SWCLK/GND/target reference and preferably NRST;
   multimeter; current-limited supplies; 3.3 V TTL USB-UART; known-good 500 kbit/s
   CAN node/interface and appropriate termination; scope/logic analyzer. Later
   motor tests additionally need independently verified energy isolation/E-stop,
   a restrained unloaded mechanism and a second operator. Keep driver motor power
   disconnected throughout A/B. Avoid back-powering the board from USB/ST-Link.
2. Check board identity/flash size, BOOT0 low, NRST, 3.3 V supply, actual 8 MHz
   clock source, SWD header continuity. Check CAN transceiver/level-shifter supplies,
   RX logic voltage compatibility, bus ground and termination before connecting.
   Measure about 60 ohms across a correctly terminated unpowered two-ended bus.
   Do not apply 24/20 V to MCU or signal rails.
3. In STM32CubeProgrammer choose ST-Link/SWD, connect under reset if necessary,
   inspect target and option bytes, load `build/SafeFlash/Bluepill-Actuator-Node.hex`,
   enable verify, program and reset/run. CLI equivalent after installing the tool:

   ```text
   STM32_Programmer_CLI -c port=SWD mode=UR -w build/SafeFlash/Bluepill-Actuator-Node.hex -v -rst
   ```

   The HEX contains its flash address. Do not mass erase or change protection/
   option bytes without reviewing their effect. These commands were not run here.
4. Connect UART RX adapter to PA9, TX adapter to PA10, and GND; use 3.3 V TTL,
   not RS-232. Terminal 115200 8N1/no flow control. Send `status` followed by newline.
   Startup banner and increasing `ms` show scheduler/service execution; inspect
   FreeRTOS task names `Motor`, `CAN-RX`, `Service`, `IDLE` in debugger if needed.
   Expected A: can=0, motor=0, wd=0, state=4, applied=0,0; fault/blocker bits reflect
   deliberately absent prerequisites. No PWM/STEP edges are expected.
5. With motor power still isolated, flash CanBench the same way. Expected can=1,
   btr=001e0003, motor=0, wd=0, state=4. If can=0 inspect blockers, reset flags,
   oscillator and transceiver first. Do not mask an initialization error.
6. Configure the separate CAN sender at 500 kbit/s. Use the production encoder
   CLI below to produce vectors, then paste its output into the sender's standard
   data-frame interface. It only prints frames and does not transmit:

   ```text
   build/host/gp_can_frame 1 600 -200 7
   build/host/gp_can_frame 2 0 0 0
   ```

   On Windows use `.exe`. The first prints `105#0100580238FF0762`. Send one frame
   every 20 ms initially. `rx` counts captured FIFO frames, `decode` structural
   successes, `reject` malformed frames, `decoded` the last structurally valid
   demand and `decoded_ms` its receive time. These do not authorize output.
   `publish` counts fresh commands actually queued for the supervisor, `stale`
   counts sequence rejects in an accepting control epoch, `barred` counts epoch/
   inhibition discards. In B, publish remains zero and barred rises. Commands
   blocked by the safety barrier cannot exercise an armed stale/timeout test.
7. Corrupt one CRC byte or transmit DLC7: reject rises, decoded/decoded_ms do not
   change. ID0x106/extended/RTR should be filtered in hardware (no rx increment).
   Encode VALID-low neutral STOP with the CLI: decode rises; a nonneutral VALID-low
   payload with corrected CRC must reject. Duplicate sequences remain barred in B;
   their stale/freshness rules are verified in host tests and must be retested in C.
8. `recover\n` queues a task-context acknowledgement. Inspect `recover=count,result`
   afterward. B restarts CAN explicitly, but supervisor result remains invalid
   because safety/actuation are unavailable. No UART command arms or sets outputs.
   Parser overflow/framing loss discards input through a newline. `uart_error` and
   `tx_drop` expose overload; responses are whole-message dropped when TX is full.
   Stop flooding UART before interpreting a missing response as task failure.
9. Bus-off/FIFO/software overflow require a controlled CAN error injector or
   debugger-assisted fault test. A receive-only node does not reliably reach bus-off
   simply by unplugging CAN (transmit error counter may not rise). Do not claim that
   test from cable removal. Observe ESR/fault history and explicit CAN restart.
   For FIFO-overrun stress use disconnected motor power and controlled bus/CPU load;
   prove overflow flags are latched and no automatic rearm occurs.

Stop on unexpected reset, panic, waveform, wrong pin activity, incorrect decoded
values, persistent CAN errors, UART corruption, rail instability or overheating.
Keep power isolated while investigating. A/B validate communication and inhibition,
not armed command freshness, interlock effectiveness or physical motor shutdown.

## Signal and motor commissioning after C prerequisites

Before energizing either motor, verify the independently effective cutoff,
default-off/reset circuitry, exact driver supply/logic/current/microstep settings,
readbacks and powered waveform test plan. Initially disconnect motors and use
appropriate loads/probes; active electronics still need verified isolation.

- Start with neutral VALID/ARM-low, then a fresh VALID/ARM/DEADMAN rising edge.
  Stream incrementing sequence numbers faster than the 200 ms timeout. Confirm
  `state=3` only after that sequence. Feed joystick movement only after this passes.
- Measure PB3 and PB4 simultaneously. Verify forward selection using a restrained,
  low-energy mechanical test; L-PWM naming is insufficient. Opposite input must
  remain at its approved inactive level for every request, fault and reset.
  `fPWM=72MHz/[(PSC+1)*(ARR+1)]`, active duty `CCR/(ARR+1)` (polarity dependent).
  At the cap the quantized duty must never exceed the approved value. Verify slew
  over real time, throttle zero, initial/reset states and no reverse pulses.
- Measure PA6/PA7 at both MCU and EM882S terminals. Check actual input amplitudes,
  pulse high/low minima, rate cap, DIR-before-first-STEP setup and reversal deadtime.
  STEP rate is ceiling(magnitude*cap/1000) Hz before timer division, which rounds
  down to a realizable rate. Check first/last pulses and live rate updates for
  runt/extra pulses. Validate real enable settling through readiness before DIR.
- Steering neutral must stop only STEP during ordinary motion; throttle neutral
  must remove only throttle. Both neutral stop both. Steering startup/reversal/
  readiness may temporarily inhibit both under Boda's preserved GP_PENDING contract.
- Test STOP, DEADMAN/ARM loss, 200 ms silence, duplicate/stale sequence, bad CRC,
  invalid-nonneutral STOP, bus-off/overflow, driver errors and E-stop. Rejected
  frames must not renew the latest accepted timestamp. Test cancellation in each
  pending phase. Record worst-case IRQ/task/critical-section and physical cutoff
  latency, rather than assuming the 10 ms motor period is a measured guarantee.
- After each genuine fault, verify old/queued commands cannot rearm. Request local
  recovery only after removing the cause, then send a **new** neutral ARM-low and
  a **new** ARM edge. Recovery alone and repeated ARM-high must not activate motors.
- With a separately approved watchdog build, stall each worker independently;
  verify refresh stops, reset occurs within measured LSI tolerances, reset flags
  identify IWDG, and the hardware remains safe throughout. Debug halt/freeze changes
  watchdog behavior; repeat without debugger. Measure all task stack high-water
  marks and sustained worst-case CAN/UART load. Fatal faults are not allowed to
  rely on a damaged kernel invoking normal motor callbacks.
- Only then connect the restrained mechanism and progressively exercise low-limit
  joystick control under supervision. Abort on unexpected direction, continued
  stepping, cap violation, unstable timing or ineffective independent cutoff.

## Git/PR handoff

713 tracked vendor files were restored to exact index/HEAD bytes after verifying
their only differences were CRLF/LF conversion. No genuine vendor edits/imports
were discarded. `.gitattributes` keeps vendor text LF. Existing staged repairs
are retained, including user-staged IDE files; review those deliberately before
the PR rather than silently removing them. Build outputs remain ignored under
`build/`. Nothing was committed, staged, pushed, merged or flashed by this task.
See `MAINTAINER_CHANGED_FILES.txt` and `MAINTAINER_VALIDATION.md` for exact changes,
results and artifacts. The source tree remains intentionally uncommitted.
