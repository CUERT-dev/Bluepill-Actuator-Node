# Dual-axis commissioning candidate

Generated for STM32F103C8T6, final schematic `hardware/final_pinout.png` and
revision-10 `Core/Inc/gp_types.h`. This is an actuation-enabled commissioning
candidate, not a physically verified operational system. No flash operation was
performed. Physical verification and powered-operation approval remain false
in `firmware-manifest.json`; neither is a compiler prerequisite.

## Artifact and reproducible build

From the repository root, with CMake, Ninja, Python and Arm GNU tools on PATH:

```powershell
cmake --preset Commissioning
cmake --build --preset Commissioning
Get-FileHash 'build/Commissioning/CUERT-Bluepill-DualAxis-Commissioning.hex' -Algorithm SHA256
```

- Profile: `Commissioning` (inherits `BenchActuationRelease`, Release).
- HEX: `build/Commissioning/CUERT-Bluepill-DualAxis-Commissioning.hex`.
- SHA-256: `afbcbf03063098be9a730855bd271f531fabc6b53a8825dfd66ceb31a571778b`.
- Flash allocation: 22,648 / 65,536 bytes (34.56%).
- RAM linker reservation: 12,472 / 20,480 bytes (60.90%), including stacks.
  This is static link allocation, not measured stack high-water usage.
- Actual compiler: STM32 Arm GCC 14.3.1, CMake 4.3.1, Ninja.
- Companion ELF, MAP, size, source fingerprint and artifact hashes are in the
  same directory. The generic `Bluepill-Actuator-Node.hex` has identical bytes.
- Bundle: `build/CUERT-Bluepill-DualAxis-Commissioning.zip` includes artifacts,
  settings, this guide and validation logs. Rebuilds may change hashes when
  source, compiler or build options change; use the newly generated manifest.

`BenchActuation` and `BenchActuationRelease` also generate the named candidate
HEX in their own build directories. `SignalTest`/`SignalTestRelease` remain
available with motor energy disconnected. Default and CAN-only builds retain
inhibited outputs. All ten profiles link.

For this machine's installed CubeIDE tools, if they are not already on PATH:

```powershell
$cubePlugins = 'C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins'
$cmakeBin = (Get-ChildItem "$cubePlugins/com.st.stm32cube.ide.mcu.externaltools.cmake.*/tools/bin" | Select-Object -First 1).FullName
$armBin = (Get-ChildItem "$cubePlugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin" | Select-Object -First 1).FullName
$ninjaBin = (Get-ChildItem "$cubePlugins/com.st.stm32cube.ide.mcu.externaltools.ninja.*/tools/bin" | Select-Object -First 1).FullName
$env:PATH = "$cmakeBin;$armBin;$ninjaBin;" + $env:PATH
cmake --preset Commissioning
cmake --build --preset Commissioning
```

## Real application and retained protection

CAN PB8/PB9 remains 500 kbit/s (36 MHz APB1, prescaler 4, BS1 15, BS2 2,
SJW 1). Standard command ID 0x105, DLC 8, sequence/CRC/flags and revision-10
payload are unchanged. The existing Blackpill -> UART -> ESP32 -> CAN path feeds
`GP_DecodeCan` -> `GpCanRx_Run` -> `GpMotor_Run` -> `CommandMap_Update` ->
`MotorDriver_Apply` -> `GpMotorHw_Write` -> real TIM2/TIM3 output adapters.
There is no standalone waveform generator, synthetic command source or automatic
arming. USART1 is diagnostics/recovery only, 115200 8N1, PA9 TX / PA10 RX.

Throttle PB3/TIM2_CH2 drives BTS7960 L-PWM; PB4 R-PWM stays inactive. Throttle
is unidirectional under the canonical unsigned 0..1000 demand contract. Steering
PA6/TIM3_CH1 supplies STEP and PA7 supplies signed DIR. Both demands can be
nonzero simultaneously. Neutral throttle removes throttle while steering
continues; neutral steering stops STEP while throttle continues. There is no
wheel-position sensor or automatic centering.

Startup is disarmed. Stream fresh valid incrementing commands, first with both
axes neutral and ARM low, then a fresh ARM rising edge with DEADMAN asserted.
Keep DEADMAN/ARM asserted while operating. Commands must remain younger than
200 ms (use the existing regular upstream stream, e.g. 20 ms). STOP, ARM/deadman
loss, timeout, stale/replayed command rejection, CAN faults/backlog, fault
latching, recovery generations and queue barriers remain intact. A genuine
latched fault needs its cause removed, local `recover` plus newline on USART1,
and a NEW neutral ARM-low / ARM-edge sequence. `status` reports CAN/motor state,
fault/block masks and requested/applied outputs; it cannot report physical power
or wiring health. No upstream repository was modified.

Boda's reversal deadtime and adapter DIR-to-first-STEP setup remain enforced.
The existing GP_PENDING contract temporarily inhibits both outputs during
steering start/reversal/setup. STOP cancels pending work. A register-only
`GpBsp_EmergencyOff` forces configured inactive GPIO levels, disconnects PWM/STEP
AF outputs, disables TIM2_CH2/TIM3_CH1 and stops both counters. Normal off and
fatal/assert/stack-overflow paths use it. Fatal shutdown requires a functioning
CPU executing the handler; CPU hangs, driver faults and damaged GPIO cannot be
made safe by software alone. IWDG remains OFF; there is no automatic CPU-stall
shutdown in this candidate.

## Single commissioning settings header

`App/Profiles/commissioning_parameters.h` contains all numeric motor
commissioning settings; `bench_actuation.h` selects that header. The full-range
limits below follow the command contract and driver/timer
constraints. They are not measured motor/mechanism approvals. CAN settings reuse
`can_500k.h`. This configuration supersedes the previous 10% / 100 STEP/s candidate.

| Setting | Candidate | Basis and limitation |
|---|---|---|
| Throttle PWM / cap | 20 kHz / 1,000 permille (100%) | Full canonical 0..1000 command range; existing PWM frequency. At 72 MHz PSC=0, ARR=3599, CCR=3600 gives true continuous active output at full command. |
| Throttle slew | 100 permille/s | Unchanged provisional ramp: ten seconds from zero to full duty, excluding setup. It does not limit steady duty. |
| Steering cap / full-scale rate | +/-1,000 permille / 166,666 STEP/s requested | Full signed command range. Rate is derived from whole-us high/low minima, not an arbitrary commissioning cap. At 72 MHz the real timer output is 166,281.755 STEP/s at full demand. |
| Steering slew | 200 permille/s | Unchanged provisional ramp: five seconds from zero to full magnitude, excluding setup. Reversal first decelerates through zero and retains deadtime/setup. |
| STEP high / minimum low | 3 us / 3 us | Manufacturer minima 2.5 us round upward in the existing integer-us interface. At full demand the timer produces 3 us high / 3.013889 us low. |
| DIR setup / reversal deadtime | 5 us / 10 ms | Existing driver/adapter contract; no shortened reversal path. |
| Enable setup | 0 us | ENA is unwired; this does not assert power or enable readiness. |
| Logic/sign selection | PWM/STEP active high, opposite PWM inactive low, positive DIR high | Intended PUL+/DIR+ level-shifted topology; mechanical direction and actual levels remain UNKNOWN. |

Both caps are now 1000, preserving the complete throttle 0..1000 and steering
-1000..+1000 command ranges. Simultaneous operation and independent neutral are
unchanged. There is no automatic steering centering.

The [Leadshine EM882S manual](https://www.leadshine.com/upfiles/downloads/df11830f96faa1fe4df379db040bb17f_1650880565511.pdf),
sections 2.1, 3.1 and 10, specifies at most 200 kHz input, at least 2.5 us STEP
high and low, and at least 5 us DIR setup. The existing API stores integer us:
rounding each phase upward gives 3+3 us and a maximum whole-Hz configured request
of floor(1,000,000/6) = 166,666 STEP/s. A 200,000 STEP/s setting would fail the
real adapter's high/low checks; it is not a usable setting without changing the
timing representation. No such redesign was made. Existing period rounding
at 72 MHz selects PSC=0, ARR=432 (433 ticks), CCR=216, so actual full-scale output
is 72,000,000/433 = 166,281.755 STEP/s. Both signs use this range. Every nonzero
permille magnitude is regression-checked for a valid timer plan.

That is an electrical command-interface range, not a rated 86CM85 mechanical
speed. Motor RPM, wheel angular rate, usable acceleration and whether steps are
lost still depend on microstepping, supply, gearing and load. Current/microstep
settings remain UNKNOWN. The unchanged slew values (100/200 permille/s), logic
polarity and mechanical direction remain provisional; DIR setup 5 us and reversal
deadtime 10 ms are retained. No physical approval or readiness measurement has
been invented. SignalTest's separate 10%/100 Hz waveform settings are unchanged
and do not constrain Commissioning.

Compile-time gates still reject missing/invalid CAN settings or decoder,
conflicting/missing output mode, unsupported routes, unknown/unselected logic
settings, zero/truncated/out-of-range motor settings and component timing
violations. Successful compilation asserts internal configuration consistency,
not that these physical properties have been verified. No five GpApproved_*
providers are required.

## Flashing instructions (not executed)

1. Isolate motor energy while programming. Use ST-Link SWDIO PA13, SWCLK PA14,
   common GND, target 3.3 V reference and NRST; BOOT0 low. Avoid conflicting board
   power supplies. The firmware retains SWD while releasing JTAG PB3/PB4.
2. In STM32CubeProgrammer select ST-LINK / SWD, connect under reset, select the
   exact named HEX, download with verification, then reset. Its addresses include
   flash origin 0x08000000; do not apply a binary-file offset to the HEX.
3. Equivalent CLI, from repository root with the programmer on PATH and NRST
   connected (syntax checked against the installed CLI's `-h`, no connection made):

```powershell
STM32_Programmer_CLI -c port=SWD mode=UR -w 'build/Commissioning/CUERT-Bluepill-DualAxis-Commissioning.hex' -v -rst
```

The installed CLI is under the CubeIDE plugins directory:
`com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_*/tools/bin/STM32_Programmer_CLI.exe`.
Check UART startup/status with motor energy still isolated. Programming success
alone does not establish driver readiness or exercise the joystick path.

## First hardware session: still UNKNOWN

These are commissioning checks, not blockers to generating the candidate HEX:

1. Establish and test independent motor-power interruption usable if firmware
   hangs. This schematic has no MCU-connected enable, E-stop/interlock or power
   readiness input. STOP stops waveforms; it does not remove motor energy or
   stepper holding torque. Do not treat the absent input as healthy. The BSP
   reports NOT_CONNECTED and `estop_ok=false` explicitly.
2. With motor energy isolated, check actual logic levels at BTS7960 and EM882S
   terminals through the installed level shifter, opposite PWM inactive level,
   STEP polarity/high/low/rate, DIR setup and reversal, first/last pulses and
   reset/boot behavior. Confirm the actual clock and CAN 500 kbit/s reception.
   PB3/L-PWM naming alone does not prove mechanical forward direction.
3. Record EM882S current and microstep DIP settings and confirm current selection
   against the installed 86CM85 motor rating. Choose supply/limits for the real
   drivers and mechanism. No invented current, microstep or safe angular limit
   has been placed in firmware.
4. Establish actual driver power/enable readiness BEFORE the fresh ARM edge and
   keep it established during operation. ENA is unwired; no readiness callback
   or artificial delay measures power stability. Follow the installed driver's
   power/enable settling requirements externally. If that condition is lost,
   use independent cutoff, restore readiness and perform a fresh startup/arming
   procedure. The MCU cannot detect that loss.
5. Restrain/unload the mechanism and progressively verify throttle direction and
   both steering directions and ranges. Abort unexpected motion. The configured
   signal-interface range does not establish achievable motor/mechanism speed.
   Record any sign/settings correction in the single header and regenerate the
   HEX/manifest.
6. Exercise diagonal joystick motion, independent axis neutral, STOP, deadman,
   200 ms silence, stale/replayed frames, reversal and recovery barriers. Measure
   output shutdown latency and reset behavior; host tests do not establish those
   physical results. Confirm the existing upstream firmware supplies the required
   flags and regular fresh sequence stream; no upstream behavior is invented.

Future independent E-stop/interlock/enable/readback support is a hardware-revision
consideration requiring real wiring and failure semantics. This candidate makes
no independent electrical shutdown claim.

## Validation delivered

- Native GCC host suite: 131/131; optimized Release suite: 131/131.
- All ten ARM presets linked and produced fresh HEX/MAP/manifests: Debug,
  Release, SafeFlash, CanBench, CanBenchRelease, SignalTest, SignalTestRelease,
  BenchActuation, BenchActuationRelease, Commissioning.
- Linked production codec/control/adapter/kernel/vector checks passed, including
  the fatal-path call to the real register shutdown.
- Strict enabled signal/bench/watchdog branch compilation passed; synthetic
  fixtures used only for compile checks, never linked into candidate firmware.
- Nine negative CMake configurations and peripheral ownership checks passed.
- Regressions retain unverified-candidate configuration and shutdown checks.
  The new actual-settings pipeline test reaches simultaneous full throttle and
  full steering in both signs, verifies true 100% PWM and the full-rate STEP
  plan, independent neutral and STOP, and checks all 1,000 steering magnitudes.
- Logs: `build/full-range-validation/checks.log`, `host-release.log`,
  and `configuration/`. Programmer CLI help was checked in the preceding build.
- No physical tests, flashing, staging, commits, pushes or merges were performed.

Nothing remains in software/configuration that prevents producing the requested
image: it has been generated. Actual simultaneous joystick-driven motor motion
still depends on the real command stream, wiring, driver settings/readiness,
direction and sufficient tested torque/rate limits described above.
