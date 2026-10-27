# Non-responsive throttle and steering: engineering report

> Latest follow-up: [STEERING_DIAGNOSIS.md](STEERING_DIAGNOSIS.md) supersedes
> the immediate-backlog policy below. The supplied hardware log shows 0x400;
> ordinary receive races now inhibit temporarily rather than latch a fault.

> Follow-up: the user confirmed the original hardware test used inhibited
> CanBenchRelease. That preset now enables commissioning actuation; see
> [CANBENCH_RELEASE_ACTUATION.md](CANBENCH_RELEASE_ACTUATION.md). The investigation
> below records the earlier configuration and artifact/test results. CanBench
> alone remains the receive-only benchmark. New ACT fields display armed,
> motor readiness, register-derived duty/STEP Hz and DIR level explicitly.

Audit date: 27 October 2026. Work performed on the existing branch, without
flashing, committing, pushing, or changing the user's IDE settings.

## A. Findings and root-cause limits

**A confirmed scheduling defect can shut down both outputs during ordinary CAN
traffic. The cause of the particular hardware session is not yet established:**
the flashed image and its supervisor status were not supplied. Successful serial
decoding alone does not establish motor permission or electrical output.

| Finding | Evidence and consequence | Disposition |
|---|---|---|
| Motor deadline can outrun CAN decoding | `GpApp_Create` previously created Motor at priority 3 and CAN-RX at 2. `GpMotor_Run` faults on `raw_count` or `rx_inflight`. When an RX interrupt and motor deadline are both ready, or a deadline interrupts decoding, Motor wins and latches `GP_FAULT_BACKLOG` (0x400). `shutdown` stops both axes, flushes commands and closes publication. | Confirmed ordering defect; fixed. `control_ready_task_order` reproduces the old ordering with one valid neutral frame, then checks the configured order through arming, demand and STOP. This is a host interleaving test, not a physical reproduction. |
| CAN-only image deliberately cannot actuate | `CanBench`/`CanBenchRelease`: actuation OFF; `GpBsp_MotorConfigure` fails closed, then `GpMotor_InitInhibited` latches dependency failure. Decoding continues, `barred` rises, `published` stays zero. `Debug` also disables CAN. | Confirmed configuration behavior, not a safety defect. Check which image was actually flashed. Preset gates remain unchanged. |
| Valid values do not imply permission | `accept_event` requires neutral with ARM low, then a fresh ARM rising edge with DEADMAN. Constant flags=7 from startup never provide the required initial ARM-low neutral. A latched fault requires local recovery before that handshake. | Expected safety behavior; preserved. Check state and flags on the board. |
| Existing reports hide the output boundary | `motor=1` reported initialization, not task execution or active outputs. No periodic supervisor state, last RX ID, accepted-command timestamp, timer registers or GPIO state were printed. | Fixed with profile identification and an additional periodic `ACT` line. |
| Hardware enables/readiness cannot be asserted by firmware | The final schematic and current BSP provide PB3/PB4 throttle and PA6/PA7 steering signals, but no MCU-controlled BTS7960 enable, EM882S ENA, E-stop or readiness input. | Physical verification required. No fictitious enable/readback or safety bypass added. |

The scheduling fix makes CAN-RX priority 3 and Motor priority 2, above the
priority-1 service task. CAN decoding remains bounded to eight frames per run.
If urgent reasons or remaining raw backlog exist after that batch, CAN-RX blocks
for one RTOS tick so the motor supervisor can handle them; simply calling
`taskYIELD` would not let a lower-priority task run. Normal notification waits
retain their 10 ms health wake. Static assertions enforce the priority order.
This follows [FreeRTOS's highest-ready-priority scheduling rule](https://github.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/04-Task-scheduling).

The backlog policy itself is unchanged. Overload, hardware FIFO backlog, and a
frame arriving during the final output-validation window can still latch a
fault. The fix addresses the normal ready-task ordering; it does not promise
fault-free behavior under every traffic interleaving. Measure task timing and
traffic on the board if 0x400 persists. Do not remove the backlog check or
automatically acknowledge faults to make a test pass.

No confirmed missing PWM-start, wrong peripheral route, disconnected command
variable, deadlock, or scaling defect was found in the active output path.
Driver supply, actual enable conditions, level-shifter bandwidth, signal polarity,
mechanical direction, current/microstep settings and wiring continuity remain
unverified. There is no evidence yet that throttle and steering have separate
hardware failures.

## B. Changes implemented

| File/function | Change |
|---|---|
| `App/Inc/gp_board_config.h` | Named CAN and Motor task priorities. No changes to output settings, caps, protocol, profiles or watchdog policy. |
| `App/Src/gp_app.c`: `GpApp_Create`, `can_entry`, `motor_entry` | Correct priority order; bounded worker handoff on fault/backlog; motor run count, last completion time and result. |
| `App/Src/gp_app.c`: `GpApp_GetStatus` | Snapshot accepted-command presence/time, shutdown reason/count, pending setup, timer registers and GPIO state under the existing lock. |
| `lib/control/src/gp_control.c`: `GpControl_CaptureIsr`; associated header | Record last captured ID/DLC even if rejected or queue-full. These diagnostic fields never grant freshness or permission. |
| `App/Inc/gp_diag.h`, new `App/Src/gp_diag_format.c` | Typed output snapshot and host-testable bounded formatter. |
| `App/Src/gp_diag.c`: `status`, `entry` | Print `ACT` after periodic CAN reports and requested status; identify selected output mode at boot. Increase interrupt-driven TX ring to 1536 bytes to accommodate paired reports. |
| `cmake/firmware/CMakeLists.txt` | Include formatter in all firmware profiles. |
| `tests/host/test_control.c`, new `tests/host/test_diag.c`, host CMake | Scheduling regression, RX metadata/freshness checks, diagnostic signedness/register formatting, maximum widths and truncation checks. |
| `tests/firmware/test_enabled_syntax.py` | Check formatter in both enabled configurations. |

Serial remains 115200 8N1 on PA9 TX / PA10 RX. Formatting runs only in the
low-priority task, never in the CAN ISR or motor callback. UART IRQ only moves
bytes through bounded rings. A full ring drops a report and increments
`tx_drop`; it never waits for the wire. Typical automatic traffic is well under
1 kB/s versus about 11.5 kB/s available at 115200 8N1. Snapshotting registers does
not prove their voltage at the pin. Timer preloads transfer on update events;
register reads need not describe the pulse currently in flight.

## C. CAN reception and command integration

The actual path is:

```text
CAN1 FIFO0 IRQ -> HAL_CAN_RxFifo0MsgPendingCallback (up to 3 frames)
 -> GpApp_CaptureFrame -> GpControl_CaptureIsr (timestamp + raw queue)
 -> notify CAN-RX -> GpCanRx_Run -> GP_DecodeCan
 -> ordered command events -> notify Motor -> GpMotor_Run / accept_event
 -> motor.latest -> CommandMap_Update -> MotorDriver_Apply
 -> GpBsp_MotorWrite -> GpMotorHw_Write -> apply_timers -> timer/GPIO outputs
```

Diagnostic `last_decoded` is a copy of the same decoder output used to publish
events. There is no separate diagnostic decoder or shadowed actuator command.
Publication additionally checks generation, acceptance, sequence and queue age.
Faulted/inhibited states still decode for visibility but bar publication.

Protocol is unchanged: 500 kbit/s, standard data ID **0x105**, DLC **8**;
bytes 0..1 sequence, 2..3 unsigned throttle 0..1000, 4..5 signed steering
-1000..1000, all little-endian; byte 6 flags; byte 7 CRC-8 polynomial 0x07,
initial zero, over the first seven bytes. Flags are VALID=1, ARM=2, DEADMAN=4.
A canonical STOP has flags=0 and both axes=0. Duplicate/out-of-order sequences
do not refresh accepted-command time. Sequence wraps modulo 65536; forward
differences must be nonzero and below 32768. Sender reset can require recovery.

All active profiles use this path and the same CAN timing (36 MHz APB1,
prescaler 4, BS1 15, BS2 2, SJW 1). The filter admits only standard 0x105 data
frames; the RX counter is not a counter of all traffic on the bus. Existing host
tests cover layout, CRC, signedness, sequence, malformed frames and real decoder
integration. The supplied hardware observation establishes reception in the
tested image, not reception/actuation in an unidentified replacement image.

## D. Throttle path

`motor.latest.throttle_pm` becomes a capped, slew-limited
`MotorOutput.throttle_duty_pm`. Zero stops that axis immediately. The driver
checks supervisor permission before writing. `GpTimer_PwmPlan` computes TIM2
PSC/ARR/CCR2; `apply_timers` enables CC2E and CEN and routes **PB3/TIM2_CH2**
through TIM2 partial remap 1. At 72 MHz and 20 kHz: PSC=0, ARR=3599,
CCR2=floor(3600*duty_permille/1000). Full duty uses CCR2=3600.
PB4, connected to R-PWM in the intended wiring, stays inactive/low in existing
profiles. PB3 is L-PWM; this does not prove mechanical forward direction.

The code deliberately starts output by setting CCER/CEN directly after HAL PWM
initialization/configuration. Absence of `HAL_TIM_PWM_Start` is not a defect.
TIM2 is a general-purpose timer and does not require TIM1's MOE mechanism.
Clock, remap, PWM1, polarity, compare preload and AF push-pull paths are present.
Both output channels are disabled and compare values cleared on full shutdown.
An independent-axis stop can leave an old CCR value while its CEN/CCER are off;
read channel enable and GPIO mode as well as CCR.

## E. Steering path

`motor.latest.steering_pm` becomes signed pulse-rate demand through the same
mapper/supervisor. It is **rate, not position**; neutral stops pulses and does
not center steering. `MotorDriver_Apply` stops both outputs briefly for steering
startup/reversal deadtime. `GpMotorHw_Write` then sets **PA7 DIR**, waits for DIR
setup across calls, computes the STEP timer plan and enables **PA6/TIM3_CH1**.
TIM3 remains unremapped. GPIO mode on PA7 prevents its unused TIM3_CH2 alternate
function from interfering. No steering ENA pin exists in this BSP.

Frequency request is ceil(max_step_hz*abs(applied_steering)/1000). Actual
frequency is `72000000 / ((PSC+1)*(ARR+1))`; active-high pulse width is
`CCR1*(PSC+1)/72000000` seconds. Both high and low minimums are checked.
PSC/ARR/CCR are preloaded; UDIS prevents mixed-plan transfers; running pulse
trains use natural updates. Startup establishes an inactive reference before
connecting AF and starting CEN. DIR reversal/deadtime/pending cancellation tests
remain active. A transient `pending=1`, result=1 and both applied outputs zero
is expected during setup; permanent pending needs investigation.

The intended PWM/STEP routes agree with [ST RM0008's remapping tables](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
The repository's [final pinout](hardware/final_pinout.png) was also inspected;
the screenshot documents intent, not installed wiring or measured electrical levels.

## F. Supervisor requirements and inhibition

States: **0 DISARMED, 1 WAIT_FOR_NEUTRAL, 2 WAIT_FOR_ARM_EDGE,
3 ARMED, 4 FAULT_SHUTDOWN**. The safety supervisor runs in the Motor task;
there is no separate safety task or mutex wait. Static task allocation, native
FreeRTOS startup, SVC/PendSV forwarding and the 1 kHz HAL/RTOS tick bridge exist.
The motor period is 10 ms. Notifications wake it early for command transitions.

| Bit / mask | Meaning and behavior |
|---|---|
| 0 / 0x0001 | STOP: immediate off, wait for new neutral/ARM handshake; not latched. |
| 1 / 0x0002 | ARM or DEADMAN loss: disarm; not latched. |
| 2 / 0x0004 | Raw queue overflow; latched. |
| 3 / 0x0008 | Event queue overflow; latched. |
| 4 / 0x0010 | CAN hardware receive overflow/read error; latched. |
| 5 / 0x0020 | CAN unavailable or bus-off; latched. |
| 6 / 0x0040 | Required E-stop input unhealthy, or explicit E-stop fault; latched. Current board has no such physical readback. |
| 7 / 0x0080 | Output adapter/driver error; latched. |
| 8 / 0x0100 | Last accepted command age >=200 ms; latched. |
| 9 / 0x0200 | Raw/event stage age >40 ms, or event receive age >=200 ms; latched. |
| 10 / 0x0400 | Undrained FIFO/raw/in-flight/event backlog, or generation changed at commit; latched. |
| 11 / 0x0800 | Mapping validation error; latched. |
| 12 / 0x1000 | Missing decoder/output permission, failed settings/startup, required watchdog failure, inhibited build, invalid/missing input provider or unsuccessful RX discard; latched. |
| 13 / 0x2000 | Output mapping interval >40 ms; latched. |

`fault` is latched faults; `history` retains prior faults after recovery; `stop`
is the last shutdown's reasons and can remain nonzero after successful recovery.
`stops` counts shutdown executions, including repeated active fault checks.
No separate brake-command field or physical brake output is implemented.
Malformed/stale packets are rejected, not used to extend freshness; continued
rejection eventually times out an accepted stream. Zero demand, slew startup,
profile caps and driver setup can also yield zero output without a fault.

Boot `block` bits are a separate namespace: 1 CAN, 2 decoder (reserved in this
path), 4 motor settings, 8 actuation disabled, 16 watchdog not started. Bit 16
is expected with IWDG OFF and alone does not inhibit these profiles. `motor=1`
and `can=1` mean initialization succeeded; neither guarantees an armed state.

All supplied presets retain IWDG OFF. The optional implementation feeds only
when both CAN and motor tasks have recent heartbeats and requires approved
reset-safe settings to enable. CPU panic/assert/stack-overflow paths stop
waveforms using direct registers, then halt (or reset only if approved).
These are not independent power interruption. The current board explicitly uses
`GP_ESTOP_NOT_CONNECTED`, not a fabricated healthy readback. SignalTest and
bench-mode software permission cannot detect whether external driver power is safe.

Safe enabling procedure, after electrical preparation:

1. Keep both axes neutral and ARM low. If state=4, remove the underlying fault,
   send `recover` plus newline through USART1, and inspect `recover=count,result`.
   Result 0 means recovery succeeded. This restarts CAN with outputs off and
   flushes old candidates; it neither arms nor injects demand.
2. Stream fresh incrementing commands at a period comfortably below 200 ms
   (for example, 20 ms). Send throttle=0, steering=0, flags=1 or 5. Observe state=2.
3. Keep neutral, assert ARM with DEADMAN so flags=7. Observe state=3 and fault=0.
4. Apply an appropriately small demand while maintaining flags=7 and sequence
   progression. Verify one axis at a time, then together.
5. STOP or permission loss requires another handshake. A latched fault requires
   explicit recovery first. Inhibited profiles cannot be recovered into motion.

## G. Verification

Completed local verification:

- **133/133 native tests passed in Debug and 133/133 in Release**, including all
  131 pre-existing cases, the new ready-task regression and the motion formatter.
  The formatter's maximum-width fixture occupies 406 of its 640-byte buffer.
- **Debug, Release, SignalTest, BenchActuation and CanBench** all configured,
  compiled, linked and generated HEX files. Each passed the existing linked
  kernel/control/vector/profile validation. The other presets were not rebuilt
  in this investigation; do not mistake their older artifacts for these images.
- Strict enabled-branch syntax checks passed for both fixture modes, including
  CAN, watchdog, tasks, interrupts, assertions and the new formatter.
- All **nine** negative CMake configuration cases passed their expected
  rejections. Peripheral ownership, CubeMX application-hook presence and
  `git diff --check` passed. The hook check inspected current sources; CubeMX
  was not rerun. Successful build logs contained no compiler warnings/errors.
- Tools: STM32 Arm GCC 14.3.1, CMake 4.3.1, Ninja and the installed native MinGW
  GCC. These are the available local tools, not a claim to have run CI's toolchain.

| Profile | Flash bytes / 65536 | Linker-reserved RAM bytes / 20480 |
|---|---:|---:|
| Debug | 30660 | 13248 |
| Release | 24152 | 13256 |
| SignalTest | 38528 | 13256 |
| BenchActuation | 38532 | 13256 |
| CanBench | 33068 | 13248 |

Static task stacks are included in reserved RAM. Compiler stack-usage reports
were inspected, but live stack margins and worst-case execution time still need
measurement. No sanitizer run or on-target timing test was performed here.
The reproducible local runner and full logs are under ignored
`build/actuation-diagnosis/verify.py` and `build/actuation-diagnosis/*.log`.
Host tests use mock clocks/callbacks and mapped storage for register shutdown;
they do not execute an STM32 peripheral or the real Cortex-M scheduler.
No board was flashed, no physical waveform was measured, and no motor was run.

## H. Flashable images and profile choice

Paths are relative to repository root
`D:\Farah's Embedded Repo\CUERT\EmbeddedSensingNode\Bluepill-Actuator-Node`:

| HEX path | Purpose |
|---|---|
| `build/Debug/Bluepill-Actuator-Node.hex` | Requested Debug artifact; CAN and actuation disabled. Do not expect motor output. |
| `build/Release/Bluepill-Actuator-Node.hex` | Requested Release artifact; CAN and both outputs enabled, software handshake required. Existing full-range provisional commissioning settings. |
| `build/SignalTest/Bluepill-Actuator-Node.hex` | **Recommended first hardware-debug image**, Debug symbols in adjacent ELF; motor energy must be physically disconnected. 20 kHz throttle capped at 10%; steering capped at 100 pulses/s. |
| `build/BenchActuation/Bluepill-Actuator-Node.hex` | Debug-symbol counterpart of the full-range commissioning output configuration, for later controlled powered investigation. |
| `build/CanBench/Bluepill-Actuator-Node.hex` | Confirm the previously working CAN path with improved visibility; outputs intentionally inhibited. |

Use the adjacent `.elf` for source debugging, not the HEX. Release is optimized
with no debug symbols. Each build directory also contains MAP, size report and
firmware manifest. Commissioning profiles additionally copy the HEX to
`CUERT-Bluepill-DualAxis-Commissioning.hex` in the same directory.

HEX SHA-256 values for the principal artifacts:

```text
Debug       163826f948bc0efa68a1268777a2eb1128d672f88ba837a1373963572a6655d8
Release     8eb059a0196d5f7b225d4a9c2ce37198247918a87792c187412f4f51bfe50b8a
SignalTest  00548ba3113e34a3b920be82ed120cbd77b62e7ddc652b7d1245157d9de6f855
```

Release/BenchActuation retain 0..1000 throttle and +/-1000 steering range;
full-scale steering plans approximately 166,282 pulses/s, not a demonstrated
mechanism speed. No new powered limits were guessed. Verify installed driver
and mechanical limits before applying these profiles with motor power.

## I. Incremental hardware isolation procedure

Start with motor energy physically isolated while retaining logic power.
Secure the vehicle before later movement tests, establish appropriate driver
current limits, provide an accessible independent emergency cutoff, and keep
the mechanism clear. Use SignalTest only in the energy-isolated stages.
Do not halt a debugger on an energized mechanism: these timers can continue
running while the CPU is halted and the watchdog is disabled.

| Stage | Test and observation | Expected result | Failure meaning / next isolation |
|---|---|---|---|
| 1. Identify image | Read boot banner and `PROFILE` on PA9, 115200 8N1. Send `status\n` on PA10 if connected. | `actuation-diag-v2`; SignalTest: can=1 actuation=1 signal=1 bench=0 wd=0. | Missing banner: verify flashed HEX, reset, logic supply, serial wiring and firmware startup. Wrong profile explains intentionally absent outputs. |
| 2. Reconfirm CAN decoding | Move joystick with energy off; watch `BLUE CAN` rx/decode/reject/drop/seq/thr/str/flags and ACT id/dlc. | rx/decode increase, id=105, dlc=8, expected unsigned throttle and signed steering. BTR=0x001e0003 for current timing. | rx flat: bus/pins/bitrate/filter/IRQ; rx rising but reject rising: layout/CRC/DLC/flags. Decode alone is not permission. |
| 3. Trace publication | `status`: compare decoded, publish/barred/stale, requested; ACT have/cmd_ms. | In enabled, fault-free firmware, publish and cmd_ms progress and have=1. | barred rising: fault/inhibited generation. stale rising: sequence issue. decoded moving with requested zero: inspect state/fault, not PWM first. |
| 4. Verify tasks | Compare ACT runs/run_ms over several seconds; watch panic and motor readiness. | runs increases at least about 100/s during normal running; notifications may increase it. run_ms remains recent. | Frozen runs: blocked/stalled task, scheduler/exception or CPU issue. motor=0: initialization failed/inhibited. Use ELF watches below. |
| 5. Verify permissions | Perform neutral ARM-low -> neutral flags=7 handshake. Check fault and state. | State 1 -> 2 -> 3, fault=0. Full neutral still produces zero output. | State 1: no exact neutral ARM-low. State 2: no valid ARM edge/deadman. State 4: decode fault mask; resolve and recover before retry. |
| 6. Throttle calculation | With SignalTest armed, request 100 permille throttle and zero steering; allow slew to settle. | requested=100,0; applied=100,0; TIM2 PSC=0 ARR=3599 CCR2=360. | Requested good but applied zero: supervisor/setup/mapper/driver. Applied good but timer off: BSP/register path. |
| 7. Steering calculation | Set throttle zero, request +100 then zero then -100 steering. | Pending briefly, then applied=0,+/-100; STEP approximately 100 Hz, DIR changes sign. Neutral stops STEP without centering. | Permanent pending: DWT time/setup progression; output fault: adapter/planner; wrong direction at terminals: inspect polarity and wiring. |
| 8. Peripheral state | Check ACT t2/t3 and pa/pb fields plus RCC/AFIO in debugger while energy is isolated. | CEN and relevant CCxE on for a nonzero axis; PWM pin AF mode; PB4 inactive. | Good compare with CCER/CEN off is not an active output. Wrong GPIO mode/remap/clock isolates MCU configuration. |
| 9. Measure MCU signals | Scope/logic analyzer PB3, PB4, PA6, PA7 against logic ground. | PB3 20 kHz/10% at settled test demand; PB4 low. PA6 about 100 Hz with >=5 us active pulses; PA7 high for positive and low for negative in current profile. Both outputs pause during steering setup/reversal. | Good registers but no signal: pin ownership, debug/SWO configuration, wrong probe pin, board damage/load. A multimeter cannot validate narrow STEP pulses. |
| 10. Measure driver inputs | Follow PB3/PB4 to BTS7960 and PA6/PA7 through the level shifter to EM882S PUL/DIR terminals; check grounds, logic power, input amplitude/current and edges against the installed driver documentation. | Waveforms arrive at the correct terminals with acceptable levels and timing. External enables are in their documented operating states. | MCU signal present but driver input absent/distorted: level shifter, wiring, loading, polarity or supply. ENA/enables cannot be fixed by an invented GPIO write. |
| 11. Shutdown/recovery, energy still off | Test zero each axis, STOP, release ARM/deadman, stop fresh CAN >=200 ms, then `recover` and handshake. Test reset with scope. | Independent neutral stops its axis; STOP/permission loss stops both. Timeout latches 0x100, outputs off, no automatic restart. Reset does not produce unacceptable pulses at driver terminals. | Any unexpected output: stop commissioning and capture the exact status/trace. Electrical default states require measurement. |
| 12. Controlled movement | Only after stages 1..11 pass, check motor supply/polarity, current and microstep settings, external enable/power settling and mechanical travel limits. Use the commissioning Debug image, neutral/ARM handshake and minimal reviewed demands; test throttle, steering, then both. | Small predictable motion, correct direction, neutral and independent cutoff effective. | Correct terminal signals with no motion points to driver power/enable/alarm/configuration, motor wiring or mechanics. Incorrect direction requires review, not blind pin swapping. |

For a scheduling stress check with motor energy off, vary command timing relative
to the 10 ms motor period, exercise both axes and log state/fault/stops for several
minutes. Healthy ordinary traffic should remain armed; overload must still shut
down. If backlog faults recur, retain the report and capture FIFO/queue depths and
task timing; do not treat the host ordering regression as a hardware stress test.

Debugger watches (use SignalTest or BenchActuation ELF; these are file-static
symbols, so qualify with the source file if your debugger requires it):

| Watch | Interpretation |
|---|---|
| `gp_boot_status` | CAN/motor readiness, blockers, reset flags, panic reason. |
| `channels.captured`, `last_rx_id`, `last_rx_dlc`, `last_decoded`, `last_decoded_ms` | ISR reception and structurally valid decoding, independent of output permission. |
| `channels.raw_count`, `event_count`, `rx_inflight`, `urgent_reasons`, `accepting`, `generation` | Where propagation stopped; true overload versus closed fault epoch. |
| `can_rx.last_sequence`, `can_rx.seen` | Sequence acceptance in the current generation. |
| `motor.state`, `latched_faults`, `fault_history`, `last_shutdown_reasons` | Current permission and historical reason. |
| `motor.latest`, `latest_received_ms`, `have_latest` | Accepted demand and freshness; ignore timestamp when have_latest=false. |
| `motor.map.previous`, `motor.applied`, `motor.output_pending` | Mapped demand, accepted driver output, setup delay. |
| `motor.driver.previous`, `last_steering_direction`, `zero_since_ms` | Reversal/deadtime progress. |
| `motor_hw.pending`, `preparing`, `stepping`, `direction`, `setup_since_us`, `timer_ready` | Actual STM32 adapter setup. |
| `motor_runs`, `motor_run_ms`, `motor_result`, `watchdog` | Task progress and last result; GP_OK=0, GP_PENDING=1, GP_INVALID=3, GP_IO_ERROR=5. |
| `motor_tcb`, `can_tcb`, `diag_tcb` | Task state/stacks via RTOS-aware debugger; investigate stack margin and starvation. |

Important registers: `RCC->CFGR` and `APB1ENR` (TIM2/TIM3/CAN clocks),
`APB2ENR` (AFIO/GPIOA/GPIOB/USART1), `CAN1->BTR/ESR/RF0R/IER`,
`TIM2->PSC/ARR/CCR2/CR1/CCMR1/CCER/CNT`,
`TIM3->PSC/ARR/CCR1/CR1/CCMR1/CCER/CNT`,
`GPIOA->CRL/IDR/ODR`, `GPIOB->CRL/IDR/ODR`, `AFIO->MAPR`,
`DWT->CTRL/CYCCNT`, `CoreDebug->DEMCR` and `SysTick->CTRL/LOAD`.

`ACT t2/t3` fields are PSC, ARR, CCR, CR1, CCER: first three decimal, last two
hex. `pa/pb` are CRL, IDR, ODR, all hex. PA7 ODR bit 7 is commanded DIR;
IDR is the sampled pin. ODR on an AF PWM pin is only the GPIO latch, not PWM
waveform readback. CEN is CR1 bit 0; TIM2 CC2E is CCER bit 4; TIM3 CC1E is
CCER bit 0. AF push-pull 50 MHz nibble is 0xB; off GPIO mode is 0x2 (2 MHz)
or 0x3 (50 MHz), depending on the stop path. PB4 is off GPIO.
TIM2_REMAP=01, TIM3_REMAP=00, CAN_REMAP=10, USART1_REMAP=0.
Do not infer JTAG/SWD state from SWJ_CFG readback: those MAPR bits have undefined
readback. Keep SWD enabled and SWO/JTAG disabled on PB3/PB4.

The retained safety logic and successful compilation are evidence about software,
not proof of physical actuator operation or vehicle readiness.
