# CUERT Bluepill Actuator Node — final software-integration brief

Status: planning handoff, 20 October 2026. **Firmware completion before flashing**. This is not a hardware-safety approval.

## Existing repository and workflow

- Project: `CUERT-dev/Bluepill-Actuator-Node`; shared integration branch `Maintaining`, user development branch `feature/gp-actuator-integration` (confirm actual local and upstream names, including possible suffixes).
- Never change `main`; do not directly push to `Maintaining`; do not commit/push/merge without approval.
- Latest locally supplied repository already contains corrected Rabe3 command mapping, corrected Boda motor driver, canonical `Core/Inc/gp_types.h` (GP API revision 10), CMake library target and 18 passing host-test cases reported and previously independently tested.
- A separate Codex run has been assigned the CAN command-management/safety foundation. **Inspect and preserve its latest changes before further work; never run overlapping agents on the same working tree.**
- The latest repository ZIP predates that CAN/safety run. It contains only baseline clock/GPIO CubeMX outputs: CAN HAL, TIM HAL, FreeRTOS middleware, ISR/task wiring, and motor hardware adapters are absent. CMake registrations must be updated with newly generated peripheral and middleware sources.

## System requirements

Blackpill ADC joystick -> UART -> ESP32 CAN gateway -> Bluepill bxCAN -> validated `TargetSample` -> safety and motor-task-controlled mapping -> motor driver -> BTS7960 forward-only throttle and EM882S stepper steering.

Only the **motor-output task** owns `CommandMap`/`MotorDriver` state and writes software motor outputs. CAN interrupt captures and timestamps frames; CAN RX task validates/sequence-checks/relays bounded ordered events. Preserve a queue-independent fault latch and urgent motor-task wakeup. Motor task runs nominally every 10 ms and additionally wakes on critical faults. Communication freshness timeout proposed 200 ms; forwarding queue residence cap proposed 40 ms per stage. These are proposals, **not measured limits**. Use locally captured frame arrival time and bounded work.

Safety: startup disarmed; fresh valid neutral with ARM low; fresh ARM rising edge to arm. On fresh invalid-neutral stop, hardware receive FIFO overrun, raw/event-queue overflow, bus-off, timeout, E-stop, output fault: fail closed, command off, clear mapping and pending transitions, then require controlled recovery. Commands already buffered before shutdown **cannot count toward rearm**. Ordinary neutral must stop the relevant axis without becoming a latched safety fault. Duplicates, stale or malformed frames do not refresh freshness. Sequence arithmetic modulo 65536, fresh forward delta 1..32767. Arm/deadman/sequence order must be preserved; reject untrustworthy backlog. Physical independent E-stop is required for powered operation and is **not** present as a confirmed circuit in the available drawings. Do not claim the DC motor physically brakes or that neutral steering returns wheels to center.

Use Ahmed's actual `GP_DecodeCan` if supplied. If absent, preserve an explicit production integration blocker; host-test mocks do not constitute a functioning CAN command system. Seek confirmation of sender/gateway handling of transient ADC-invalid packets: a single stop can be lost upstream despite later valid sequence numbers.

## Newest hardware reference: yellow Altium schematic, provisional pins

| Function | Board pin | Candidate peripheral |
|---|---|---|
| CAN RX | PB8 | bxCAN remap |
| CAN TX | PB9 | bxCAN remap |
| Steering STEP/PUL | PA6 | TIM3_CH1, default mapping |
| Steering DIR | PA7 | GPIO |
| BTS7960 L-PWM | PB3 | TIM2_CH2 with appropriate partial/full TIM2 remap |
| BTS7960 R-PWM | PB4 | GPIO inactive **only if PB3/L-PWM is approved forward** |
| SWD | PA13/PA14 | SWD enabled, JTAG disabled (frees PB3/PB4) |

**Stop before energizing hardware:** The proposed PB3-PWM arrangement is viable only if L-PWM produces the team's required *forward* motor direction. If R-PWM/PB4 is required forward, do not simultaneously allocate PA6 and PB4 as TIM3_CH1; obtain a hardware-approved pin/timer reassignment. Never guess driver input polarity, pulse/setup times, rated frequency, enabling circuitry, CAN bitrate, motor caps, 20 V throttle supply arrangement, or E-stop wiring. The newest schematic seems to tie BTS7960 EN high and leave EM882S ENA unconnected. Therefore firmware **cannot claim an independent motor enable cutoff**. The stepper off callback must have clearly defined pulse-off/holding-torque behavior.

## What to complete after the in-progress CAN/safety run

1. Inspect actual newly modified code and outstanding blockers from Codex's CAN/safety task.
2. Generate/ integrate CubeMX peripheral configurations for bxCAN, TIM2, TIM3, GPIO, SWD and FreeRTOS. Observe pin remap collision and FreeRTOS ISR priority restrictions; reserve debug capability. Prefer CubeMX-generated initialization; keep application logic outside regeneration-owned sections. Delay activating IWDG until reset-safe outputs/supervision strategy is specified. Do not silently substitute fabricated HAL drivers.
3. Define fixed-size nonblocking BSP callbacks for BTS7960 PWM and EM882S STEP/DIR per Boda's `MotorWriteFn`/`MotorOffFn`. `GP_PENDING`: both outputs must remain inactive while deadtime and direction/enable setup are unresolved; never self-activate asynchronously between calls. Hardware STEP pulse train generated by timer, not toggled in task loops. `MotorOffFn` cancels pending transitions and output waveform synchronously as far as firmware can verify. Confirm approved setup delays and GPIO electrical states before enabling outputs.
4. Wire actual application creation/ISR callbacks and task notification/queue mechanics to `GP_DecodeCan`, safety logic, `CommandMap_Update` and `MotorDriver_Apply`; no firmware calls to a mock decoder.
5. **Safe-default build**: all physical motor outputs inhibited at startup/reset, and remain inhibited without explicitly approved settings/hardware-safe enable mechanism. Missing dependencies must fail build with a clear diagnostic or prevent actuation; never fall back to unsafe placeholder values. Do not power motors or flash.
6. Preserve and extend deterministic host tests (including STOP/rearm burst, stale frames, tick rollover, queue overflow, CAN bus-off, GP_PENDING, failed callback, and interrupt/task handoff), compile both libraries together, run firmware cross-compile, and report `.elf`, `.hex`, `.map`, SHA, RAM/flash usage, tests and any remaining configuration decisions.

## Definition of completion

**Software-complete, safe-disabled** means: all available real modules compile and link, all host tests pass, correct files generated and CMake registration complete, no unresolved mock linked into live control, no nonzero actuation possible by default, configuration gaps clearly documented, and a hardware commissioning checklist exists. This does **not** mean powered-motor-ready or vehicle-safe.

**Hardware-commissioned** is a separate milestone requiring approved wiring, measured PWM/STEP/DIR waveforms and polarity, setup and reversal timing, real fault/STOP cutoff measurements, watchdog/reset behavior and independent physical E-stop verification.
