# Independent Bluepill audit — 21 October 2026

> Subsequent refactor: [CURRENT_BOARD.md](CURRENT_BOARD.md) supersedes the
> callback blockers and profile/build totals below. Real unpowered SignalTest
> images now link without those interfaces; powered operation remains unsupported.

The Bluepill software processes the two axes of one joystick independently and
simultaneously in the production codec/control/mapper/driver/timer-planning path.
The independent audit found and fixed two software defects. The resulting
software is ready for review and for the first inhibited communication bench
tests. **It is not an operational motor image or a physically verified system.**
No firmware was flashed and no target, UART, CAN bus or motor was exercised.

This report supersedes the test totals, image sizes and no-defect conclusion in
`MAINTAINER_VALIDATION.md`. The current working tree, rather than HEAD or that
earlier report, was inspected. The branch is `feature/gp-actuator-integration`;
the PR target remains **Maintaining**.

## A. Confirmed defects and corrections

### High: accumulated slew bypassed the intended restart ramp

`lib/control/src/gp_control.c`, `GpMotor_Run`, advanced `CommandMap_Update` on
every motor period while `MotorDriver_Apply` returned `GP_PENDING` and held
both outputs off. On completing steering readiness, the driver applied all that
unapplied ramp progress at once. This also affected ordinary setup/reversal
waits; a longer readiness delay made the jump larger.

The new `pipeline_readiness_slew` case links the actual production codec,
supervisor, mapper, Boda driver and hardware planner. With a synthetic
1,000 permille/second slew, a 10 ms period and a 200 ms readiness wait, the
baseline first applied **230 permille on both axes**, instead of the allowed
10-permille restart increment. The old source was recovered into an ignored
test file from the previous artifact archive and its SHA-256 was verified
against this audit's baseline before reproducing the failure.

The supervisor now resets unapplied integer slew history on `GP_PENDING`,
retains the fractional credit from before the attempted update, and tracks the
pending state. An early throttle-neutral event leaves an already-off steering
readiness transaction pending; steering neutral and global STOP still cancel
it. Fractional credit is retained so a 1-permille/second setting can actually
complete readiness. The next applied output ramps from zero. Boda's intentional
both-output pause and all public revision-10 types/functions are unchanged.

Regression evidence: `pipeline_readiness_slew`, `pipeline_readiness_fractional`,
`pipeline_pending_fractional_neutral`, `pipeline_readiness_stop`,
`pipeline_reversal_failure`, and the existing control
neutral/reversal cases. Existing expected values that explicitly included the
unapplied ramp were corrected, without removing their behavioral assertions.
Before-fix evidence: `build/independent-audit/slew-before.log`.

### Medium: malformed UART text could queue recovery

`App/Src/gp_service.c`, `GpService_Byte`, discarded every carriage return,
including those inside a command. Thus `reco<CR>ver<LF>` became `recover`.
A native reproduction returned `GP_SERVICE_RECOVER` (2) instead of
`GP_SERVICE_BAD_COMMAND` (3).

The parser now removes only one final CR when processing LF. Embedded, leading
and repeated CR bytes remain invalid command text. The service regression
retains LF and CRLF acceptance and verifies malformed variants, overflow,
invalid characters and error recovery. This defect did **not** bypass the
supervisor's neutral/ARM-edge requirements; its consequence was an unintended
recovery request, including a possible CAN restart while faulted.
Before-fix evidence: `build/independent-audit/parser-before.log`.

## B. Reproduced tests and builds

Tools used: ARM GCC 14.3.1, native MinGW GCC 8.1.0, CMake 4.3.1, Ninja,
Python 3.13, MSVC 19.43 AddressSanitizer, CubeMX 6.18.1 and the installed
STM32Cube F1 V1.8.7 package. The CI-pinned ARM 14.2.Rel1 and remote CI were
not run. Tools were invoked by their installed absolute paths or through the
task-local PATH; no tool installation or global configuration was changed.

| Check | Actual result |
|---|---|
| Baseline host CTest | 110/110 passed before changes |
| Final native host CTest | **116/116 passed** |
| Optimized native Release CTest | **116/116 passed**; assertions retained |
| MSVC ASan | **106/106 runtime cases passed**, no sanitizer findings; the other 10 CTest cases are compiler configuration gates |
| Debug, Release, SafeFlash, CanBench, CanBenchRelease | All five ARM images compiled and linked successfully after fixes |
| ELF checks | Unique strong codec and CAN callbacks, kernel/control symbols, exception/CAN/UART vectors and naked exception forwarding passed for all five images |
| Profile symbol checks | Active motor adapter absent from all delivered images; actual HAL CAN initialization/filter/start/notifications present in both CAN bench images |
| Enabled hardware branch check | Strict ARM syntax check passed; **not** an enabled-image link |
| GCC analyzer | 11 application/portable translation units compiled with `-fanalyzer -Wall -Wextra -Werror -Wpedantic`, zero diagnostics, including synthetic enabled BSP/tasks/interrupts |
| CMake negative gates | All 4 expected rejections passed; host suite also passes all 10 header configuration gates |
| CubeMX | Actual generation in an isolated directory succeeded; hooks and peripheral ownership checked in the **generated `Src` files**, not untouched input copies |
| Repository checks | Staged and unstaged `git diff --check` passed; index unchanged; vendor checks below passed |

MinGW sanitizer probes could not link `-lasan` or `-lubsan`. ASan was then run
successfully using the installed MSVC compiler and available SDK headers/libs.
Its compile log contains old SDK C5105 warnings and the intentional uint8 mask
conversion C4310 in the codec; these are not runtime ASan findings. **UBSan
remains unavailable** with the installed usable C compilers; **cppcheck is not
installed**. These checks are not represented as passing.

The following are the commands used, with tool installation prefixes omitted:

```text
cmake -S tests/host -B build/independent-audit/host-ninja -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/CodeBlocks/MinGW/bin/gcc.exe"
cmake --build build/independent-audit/host-ninja
ctest --test-dir build/independent-audit/host-ninja --output-on-failure
cmake -S tests/host -B build/independent-audit/host-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER="C:/Program Files/CodeBlocks/MinGW/bin/gcc.exe"
cmake --build build/independent-audit/host-release
ctest --test-dir build/independent-audit/host-release --output-on-failure
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
cmake --preset SafeFlash
cmake --build --preset SafeFlash
cmake --preset CanBench
cmake --build --preset CanBench
cmake --preset CanBenchRelease
cmake --build --preset CanBenchRelease
cmake --build build/Debug --target gp_check_enabled_syntax
python tests/firmware/test_configuration.py --cmake <installed-cmake.exe>
python scripts/check_peripheral_ownership.py
python scripts/check_cubemx_hooks.py --src build/independent-audit/cubemx/Src
python scripts/check_peripheral_ownership.py --root build/independent-audit/cubemx --src Src
git diff --check
git diff --cached --check
```

Every ARM build invokes `scripts/check_firmware.py` on its linked ELF and
`scripts/firmware_manifest.py`. Exact expanded commands and logs are in
`build/independent-audit/`: `baseline-checks.log`, `postfix-checks.log`,
`host-release.log`, `analyzer.log`, `asan/build-run.log`, `sanitizer-probe.log`,
`cubemx-isolated.log`, and the `run-*.py` scripts. ASan uses
`cl /std:c11 /fsanitize=address /Zi /Od /UNDEBUG /W4`, then executes every runtime
case discovered from CTest. The scripts and logs are included in the audit bundle.

CubeMX's first invocation could not initialize its default Java home in this
sandbox. The successful rerun used a task-local Java home and seeded the output
layout that CubeMX actually generates. The source repository was not regenerated
or rewritten. A generated-project ARM link is not claimed.

| Profile | Flash used / 65,536 bytes | RAM reserved / 20,480 bytes |
|---|---:|---:|
| Debug | 28,452 | 12,464 |
| Release | 18,452 | 12,320 |
| SafeFlash | 28,452 | 12,464 |
| CanBench | 30,836 | 12,464 |
| CanBenchRelease | 19,764 | 12,320 |

RAM includes static task stacks and linker reservations. It is not measured
peak stack usage. Each current image has ELF, HEX, MAP, size and SHA manifests
under `build/<profile>/`. Earlier `maintainer-artifacts.zip` and earlier reports
describe the pre-audit firmware; use `build/independent-audit-artifacts.zip` for
this result.

Final firmware source fingerprint (all five profiles):
`381be658c9aad1ed70fb6fa36e93ef4682e7eb3cce5f7a57e27513f758149d32`.

## C. Actual profiles and execution path

`main` runs HAL/clock initialization, `GpBsp_Init` and `GpApp_Create`.
`GpApp_Create` initializes channels with the real `GP_DecodeCan`, creates the
CAN and motor tasks, starts the UART service and starts native FreeRTOS.
The motor task establishes off/inhibited state before enabling CAN interrupts.

| Profile | What initializes and what is permitted |
|---|---|
| Debug / Release / SafeFlash | Native FreeRTOS, UART 115200 8N1 and real inhibited supervisor. CAN start, actuation and IWDG are OFF. Motor pins PB3/PB4/PA6/PA7 are analog/high impedance, timers/channels are stopped. This does not establish electrically safe levels at powered drivers. |
| CanBench / CanBenchRelease | Same, plus real bxCAN at 500 kbit/s, PB8/PB9, hardware standard-data-ID filter, real HAL RX/error callbacks and production decoding. Inhibited supervisor keeps dependency/E-stop permission faults; `publish=0`, requested/applied remain zero. Structurally decoded values and counters are observable. `recover` restarts CAN but cannot clear missing motor prerequisites. |
| Motor-enabled custom image | Source implementation exists and enabled branches compile. No legitimate board provider/approval header exists for the present wiring; this image was not linked. The gates remain intact. |

The actual RX chain is `USB_LP_CAN1_RX0_IRQHandler` -> `GpBsp_CanInterrupt` ->
`HAL_CAN_IRQHandler` -> `HAL_CAN_RxFifo0MsgPendingCallback` ->
`GpApp_CaptureFrame`/`GpControl_CaptureIsr` -> bounded raw ring and task
notification -> `GpCanRx_Run`/`GP_DecodeCan` -> ordered events/sticky faults ->
`GpMotor_Run` -> `CommandMap_Update` -> `MotorDriver_Apply` ->
`GpBsp_MotorWrite`/`GpMotorHw_Write` -> `apply_timers`/`commit_timer` -> TIM2,
TIM3 and GPIO. The path beyond the inhibited supervisor is gated off in A/B.

CAN clock evidence: HSE_VALUE=8 MHz, HSE PLL x9, AHB /1, APB1 /2, APB2 /1 in
the actual startup code and `.ioc`; timer clock is 72 MHz. CAN uses
36 MHz / [4*(1+15+2)] = 500 kbit/s, sample point 16/18 = 88.889%, SJW 1,
normal-mode BTR `0x001e0003`. Runtime timing validation checks the clock equation.
The filter masks ID, IDE and RTR; DLC is checked by the decoder, which requires
exactly eight bytes. Layout, signed little-endian steering, flags and CRC-8
0x07/init 0/no reflection/xor 0 over bytes 0–6 match the stated contract.
VALID-low neutral with flags zero is a valid STOP, not malformed data.

The decoder does not update accepted-command freshness on CRC/range/flag/format
failure. Sequence comparison accepts modular deltas 1..32767, rejecting zero,
older and half-range deltas; rollover is tested. Rejected/stale frames do not
consume a new sequence or refresh motor permission. STOP uses the urgent latch
even if the event queue is full. Shutdown discards peripheral/software backlog
and increments a generation; an old in-flight decode cannot cross recovery.
Explicit fault recovery still needs new neutral ARM-low followed by a fresh
ARM rising edge. The protocol has no sender clock/authentication: “fresh” means
the local receive-age and sequence rules, not proof against deliberate replay
across a new recovery generation.

Final pin routes agree with the STM32F103 pin functions: PB3/TIM2_CH2 partial
remap 1 for L-PWM; PB4 remains inactive GPIO; PA6/TIM3_CH1 without TIM3 remap for
STEP; PA7 GPIO DIR; PB8/PB9 CAN remap 2; PA9/PA10 USART1; PA13/PA14 SWD with
JTAG disabled. There is no PB4/TIM3 remap conflicting with PA6. The `.ioc` owns
clocks/SWD; the gated BSP/service owns the other peripherals. PB3/L-PWM is
**not** evidence of mechanical forward direction. The existing schematic review
was retained. Pin-function source: [ST STM32F103x8/B datasheet, tables 5 and pin descriptions](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf).

## D. Motor, concurrency and diagnostics evidence

Throttle is unsigned 0..1000 at the protocol boundary; invalid range is rejected,
then valid demand is clamped to the configured cap. The driver rejects negative
throttle. The planner floors duty to representable timer ticks; PWM uses mode 1
and explicitly approved polarity. Steering is a signed rate, not position:
rate Hz = ceil(max_step_hz * abs(permille) / 1000), followed by timer division
that does not exceed that frequency. High/low counts round upward and infeasible
plans fail closed. Ordinary axis-neutral requests clear only that axis; both
neutral stop both without requiring a new ARM edge. Steering neutral does not
center wheels. Safety STOP/faults clear both axes and pending work.

Timer code uses buffered PSC/ARR/CCR, UDIS around multi-register updates and
natural updates for a running waveform. Initialization does not start PWM.
STEP begins from CNT=ARR with its reference inactive before AF connection.
Direction changes follow Boda deadtime, optional readiness, then DIR setup.
Register sequencing and arithmetic were reviewed; timer/GPIO electrical edges,
first/last pulse widths, polarity and physical direction remain unmeasured.

The readiness phase calls `GpApproved_ReleaseInhibit` and
`GpApproved_PrepareSteering` before writing DIR. These functions are declarations
only: no real signal/readback implementation exists. `GP_PENDING` retries without
a busy wait and holds both waveforms off; `off` cancels the transaction. The
subsequent `max(dir_setup_us, enable_setup_us)` delay is an internal setup delay,
not an ENA output. EM882S can default enabled with ENA disconnected; waiting in
software does not establish a power or enable transition. The manufacturer's
manual gives 2.5 us pulse width, 5 us DIR setup, optional ENA and a 200 ms enable
interval; the implementation conservatively rounds pulse minima to 3 us and
requires a real provider for readiness. See [Leadshine EM882S manual, sections 3.1 and 10](https://www.leadshine.com/upfiles/downloads/df11830f96faa1fe4df379db040bb17f_1650880565511.pdf).
The BTS PWM ceiling of 25 kHz agrees with its [Infineon datasheet](https://www.infineon.com/dgdl/BTS7960_Datasheet.pdf?fileId=db3a304412b407950112b43945006d5d&folderId=db3a304412b407950112b408e8c90004).
Neither source establishes the assembled module's safe settings or mechanical
braking. Stopping STEP does not establish removal of holding torque.

| Claim | Evidence achieved | Missing evidence |
|---|---|---|
| Boot/scheduler/startup | Implemented; ARM compiled/linked; vector/forwarder checks | No target execution |
| Real 500 kbit/s CAN receive | Implemented; real HAL linked in B; timing/filter/ISR source inspected | No bus/target test; HAL ISR not executed by host fixtures |
| Both joystick axes decoded and processed | Real codec/control/mapper/driver/planner host-linked and tested together; firmware codec/control linked | Armed target processing gated off in A/B |
| Correct capped/ramped demands | Production host tests, including new restart-ramp regression; ARM compiled/linked portable policy | Approved machine parameters and target waveform measurements |
| Independent neutral and diagonal movement | Production full-path host tests pass | Physical outputs unverified |
| PWM and STEP/DIR hardware | Planner host-tested; actual enabled BSP compiled/analyzed | Active adapter excluded from delivered images; no operational link or scope tests |
| Global shutdown and pending cancellation | Production supervisor/driver/planner host tests; inhibited off path linked | Physical inhibit/cutoff implementation and validation |
| Recovery without queued commands rearming | Host sequence/generation/fault regressions pass; real CAN restart and service linked | Armed target fault injection and readback |
| Running actual motor adapters | Source connected; enabled ARM branch compilation succeeds | Board callbacks/settings missing; cannot run with present wiring |

Concurrency review: motor priority 3, CAN priority 2, diagnostics priority 1;
static stacks are 512/384/768 words, idle 128 words. The motor has an absolute
10 ms deadline, command timeout 200 ms, queue-stage and mapper maximum interval
40 ms. Raw/event capacities and per-run budgets are 8 (storage maxima 16).
CAN ISR drain budget is 3; software overflow latches a fault independently of
queue space and disables RX notifications until explicit restart. Hardware
overrun/bus-off is latched before HAL clears flags. CAN priorities 6 are legal
for FreeRTOS syscall threshold 5; UART priority 7 uses no RTOS API in the ISR.
Notifications remain pending across task waits. Every shared channel update and
diagnostic snapshot is protected by save/restore PRIMASK and barriers. Only the
motor task performs normal output writes; fatal exceptions rely on approved
reset-safe hardware rather than calling arbitrary drivers from a damaged stack.

No mutex-based priority inversion or dynamic task allocation is present. The
final permission check/output commit is interrupt-masked; its arithmetic,
bounded FIFO flush and eventual board callbacks still need measured execution
time. Heavy traffic can trigger conservative backlog shutdown. No hard real-time
guarantee, immunity to IRQ starvation, or measured stack margin is claimed.
Compiler stack reports were inspected (e.g. motor entry 88 bytes, motor run
120 bytes, UART status 360 bytes excluding descendants/library calls). They
are not call-chain maxima or stack high-water measurements.

Watchdog readiness requires both worker timestamps and rejects expired/missing
progress, including rollover tests. A stalled CAN or motor task cannot keep
the normal feed path healthy indefinitely. IWDG is nevertheless OFF in every
delivered profile; fatal-fault/reset and task-stall safety remain hardware and
commissioning prerequisites for a separately approved watchdog build.

UART is direct-register USART1 at 72 MHz / BRR 625 = 115200, 8N1, PA9/PA10.
RX/TX rings are 64/768 bytes; the parser line is 16 bytes, response buffer 640.
The low-priority task consumes at most 16 RX bytes per wake and queues whole
responses without waiting for transmission. Formatting occurs outside the
critical section; publishing TX head is serialized with its ISR. Overflow or
framing loss discards through newline; counters expose loss. CAN has higher IRQ
and task priority. This is bounded source behavior, not a measured CAN-load or
UART electrical result. Diagnostic counters describe structural decoding
separately from permission; generation flushes can discard queued frames before
decode, so `rx=decode+reject` is not a general conservation invariant. `recover`
only requests motor-task handling and cannot inject a motor demand.

## E. Remaining blockers and smallest actions

| Missing behavior | Present wiring / minimum concrete action | What it blocks |
|---|---|---|
| `GpApproved_EstopHealthy` | No independent E-stop/readback is established. Supply an independently effective cutoff circuit and fail-safe status input with reviewed polarity. | Operational output build and powered motor use; not inhibited CAN/UART bench |
| `GpApproved_InterlockHealthy` | No ready/interlock feedback exists. Supply actual permission/readiness feedback and distinguish permission-to-enable from already-enabled state. | Same |
| `GpApproved_Inhibit` / `GpApproved_ReleaseInhibit` | BTS enables tied high and EM882S ENA unwired provide no MCU-controlled independent inhibit. Add a verified inhibit/release interface with default-off behavior. | Same; current A/B cannot be repurposed as active signal-test images by setting a flag |
| `GpApproved_PrepareSteering` | No enable/power-ready observation exists. Wire enable/readback or establish a reviewed external stable-power/enable interface and implement cancellable, timestamped readiness. | Enabled STEP/DIR and powered steering; not CAN bench |
| Motor settings and direction | Confirm whether PB3/L-PWM is forward, signal polarities/levels, driver settings, caps, PWM/STEP rates, slew and reversal limits. Supply real approval header/provider sources. | Active signal and motor tests; does not block inhibited communication |
| Reset/watchdog behavior | Verify reset/power sequencing and external inactive levels. Set watchdog timing from measured LSI bounds and test task stalls if enabling IWDG. | Powered operation/safety claim, not motor-energy-isolated CAN bench |
| Target evidence | Operator performs first SafeFlash/UART and CanBench/CAN tests with motor energy isolated, then scope and fault tests after active-output prerequisites exist. | Any claim of physical success |

The missing hardware functions cannot be replaced with constant healthy values,
no-op inhibit callbacks or a software delay. None was supplied. No remaining
confirmed software defect independent of these interfaces was left unfixed.
This is an audit conclusion within the tests above, not proof of defect absence.

## F. Repository and maintainer readiness

HEAD remains `b77a3c24822c4722dc4f9ab325edbb86ea09073c`. No staging, commits,
pushes, merges, resets, cleans or flashing were performed. The original index
SHA-256 and existing staged content are unchanged. All **718 tracked Drivers
files** match index bytes exactly and all **36 imported vendor hashes** match
`docs/vendor-manifest.json`. The 713 cached vendor modification entries visible
in Git status contribute no content diff; no vendor restoration was needed in
this audit. The historical act of restoring 713 files cannot be independently
reconstructed from Git status, but the resulting bytes were checked directly.

This audit changes `App/Src/gp_service.c`, `lib/control/src/gp_control.c`,
`lib/control/include/gp_control.h`, the host CMake harness and the service/control
tests; it adds `tests/host/test_pipeline.c` and this report. Documentation entry
points now refer here. Boda's driver, Rabe3's mapper, protocol API, electrical
gates and other repositories were not changed by this audit.

The baseline includes staged repairs, untracked production/vendor imports and
IDE metadata. Preserve and review that whole candidate deliberately; a PR of
HEAD alone would omit the working firmware. Final exact status and file/hash
inventories are in `build/independent-audit/final-status.txt`, `baseline.json`
and `integrity.json`. Generated binaries/logs remain under ignored `build/`.
Final raw status contains 818 entries: 713 are the cached vendor entries
described above. The 15 original staged entries are preserved, and 84 files are
untracked, including existing production/vendor imports and the two new audit
files. Raw status is not a count of this audit's edits: this audit changes eight
existing files and adds two.

**Ready for a software-review PR into Maintaining with hardware blockers stated;
not ready to approve powered motor operation.** The next achievable system
milestone is physical inhibited UART/CAN bench validation, followed by real
board safety interfaces and active-output commissioning.
