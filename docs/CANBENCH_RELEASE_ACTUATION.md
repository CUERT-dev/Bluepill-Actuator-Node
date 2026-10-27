# CanBenchRelease: CAN and full-range actuation

> Superseded operator policy: the user subsequently requested direct actuation.
> CanBenchRelease now uses `GP_DIRECT_COMMANDS=ON`: no ARM, DEADMAN or neutral
> arming sequence. See [DIRECT_COMMANDS.md](DIRECT_COMMANDS.md) for the current
> behavior, tests and HEX checksum. The report below describes the preceding build.

27 October 2026. The user confirmed the successful physical CAN test used the
previous **CanBenchRelease**, which explicitly disabled actuation. That explains
why neither motor responded despite changing decoded values. This change makes
the preset actuation-capable using the existing commissioning implementation.
It does not establish physical readiness.

## Old and new settings

| Setting | Previous CanBenchRelease | New CanBenchRelease |
|---|---|---|
| Inheritance | `release-base` plus CAN-only options | `BenchActuationRelease` |
| Build type | Release | Release |
| CAN | Enabled | Enabled, unchanged implementation |
| Profile | `can_bench.h` | `bench_actuation.h` -> `commissioning_parameters.h` |
| CAN timing header | `can_500k.h` | Same `can_500k.h` |
| Actuation | Disabled | Enabled, controlled bench commissioning mode |
| Throttle | No motor output | 0..1000 permille, up to 100% duty |
| Steering | No motor output | -1000..1000 permille pulse-rate demand |
| SignalTest mode | Off | Off; no 10% SignalTest amplitude cap |
| Watchdog | Off | Off, existing policy preserved |
| USART1 | 115200 8N1 diagnostics | Same transport, expanded output fields |
| Physical validation | Not established | Still not established |

**CanBench remains receive-only**, using `can_bench.h` with its prohibition
against actuation intact. No guards were removed. The existing profile headers,
`gp_board_config.h` and initialization logic already support the selected
actuation configuration and needed no further changes for this request.

## CAN and physical output paths

CAN remains PB8 RX / PB9 TX, 500000 bit/s. At 36 MHz APB1: prescaler=4,
BS1=15 TQ, BS2=2 TQ, SJW=1 TQ; BTR=0x001e0003. The filter still accepts
standard data ID 0x105. DLC=8, sequence, throttle, signed steering, flags,
CRC and little-endian layout are unchanged. `gp_stm32_bsp.c`, `gp_can_codec.c`,
`can_500k.h` and the protocol header were not modified for this request.
The previous task-ordering correction is retained. Physical CAN reception in
this new image still needs an on-board check.

Both axes follow ISR -> raw queue -> `GpCanRx_Run` / `GP_DecodeCan` -> ordered
events -> `GpMotor_Run` / `motor.latest` -> `CommandMap_Update` ->
`MotorDriver_Apply` -> `GpMotorHw_Write` -> timer/GPIO adapter. Permission is
checked before outputs are committed.

| Axis | Existing software route | Active configuration |
|---|---|---|
| Throttle | PB3 / TIM2_CH2; PB4 opposite input held low | TIM2 partial remap 1, active-high PWM1, 20 kHz; PSC=0, ARR=3599, CCR2 up to 3600 |
| Steering | PA6 / TIM3_CH1 STEP; PA7 GPIO DIR | TIM3 unremapped, active-high STEP; DIR high for logical positive |

`GpBsp_Init` enables AFIO, GPIOA/B and TIM2/3 clocks and establishes off
outputs. `GpBsp_MotorConfigure` validates the adapter and initializes/configures
the HAL timer channels. `apply_timers` explicitly sets CCER/CEN after loading
plans and routing AF push-pull pins; `HAL_TIM_PWM_Start` is not missing.
Preloads, startup updates, direction setup and reversal handling remain intact.
Register shutdown disables channels/counters and forces PWM/STEP inactive.
Source/link checks establish the software path, not measured pin behavior,
mechanical direction or installed electrical compatibility.

## Full range, slew and unresolved steering speed

Both caps remain **1000 permille**. Existing full-pipeline tests reach full
throttle and positive/negative full steering through the real codec, supervisor,
mapper, driver and timer planner. No arbitrary amplitude cap was introduced.

Throttle slew remains **100 permille/s**, approximately **10 seconds** from
zero to full duty. Steering slew remains **200 permille/s**, approximately
**5 seconds** from zero to full magnitude. At 10 ms intervals the increments
are approximately 1 and 2 permille. Small initial outputs after joystick
movement are therefore expected. Neutral and safety shutdown stop immediately.
A full steering reversal decelerates to zero, waits for deadtime/setup, then
accelerates oppositely: about 10 seconds plus setup. Steering setup can briefly
pause both axes. Neutral does not automatically center steering.

**No validated motor/mechanism STEP maximum is recorded in the repository or
has been supplied for this change.** The retained `GP_MAX_STEP_HZ=166666` is
the existing electrical waveform ceiling for 3 us high/3 us low timing, not a
validated mechanical speed. Full demand plans PSC=0, ARR=432, CCR1=216,
approximately **166281.755 pulses/s**. This artifact is for **unpowered waveform
verification until installed hardware limits are validated and configured**.

Once a validated maximum is available for the actual motor, microstep setting,
gearing and mechanism, configure `GP_MAX_STEP_HZ` in
`App/Profiles/commissioning_parameters.h` accordingly, within the existing
electrical timing checks, and rebuild/retest. Keep `GP_STEERING_CAP_PM=1000` so
full normalized command maps to that validated maximum. This shared header
affects all commissioning presets. The driver's advertised limit alone is not
a mechanical limit. Current configuration is external to this firmware and was
not altered. No new physical approval or healthy readback was fabricated.

## Diagnostics and safety requirements

Existing `BLUE CAN` reports retain decoded throttle/steering, sequence, flags,
counters and CAN state. The nonblocking `ACT` report adds:

- `armed`: 1 for supervisor state 3; `motor`: initialization succeeded.
- `pwm_pm`: active register-derived duty, 1000 = 100%.
- `step_hz`: register-derived pulse frequency truncated to whole Hz; an actual
  99.999 Hz waveform reports 99, not 100.
- `dir`: PA7 output latch level, not a claim about mechanical direction.

Requested/applied values, pending setup, faults, shutdown reason, task progress,
raw timer registers and GPIO snapshots remain available. Disabled channels
report zero duty/frequency even if stopped registers retain old compare values.
Preloads can describe the next pulse rather than the current pulse. These are
register calculations, not voltage measurements. Calculation/formatting runs in
the priority-1 diagnostic task; CAN IRQs do not format text and UART congestion
drops reports rather than blocking motor control.

Outputs require successful initialization, no latched fault, fresh valid neutral
with ARM low, then an ARM rising edge with DEADMAN. Keep neutral through that
edge; observe state=3, armed=1, fault=0 before demand. Stream incrementing
commands comfortably faster than the **200 ms** timeout. STOP, permission loss,
malformed/stale rejection, queue/freshness checks, latching, explicit recovery,
direction handling and timer/GPIO shutdown remain unchanged. Resolve faults,
send `recover\n`, inspect its result, then repeat the handshake. Recovery does
not arm or inject demand. The board lacks MCU-connected enable/E-stop/readiness
readback; software does not provide independent power isolation.

## Files changed for this request

| File | Reason |
|---|---|
| `CMakePresets.json` | Reuse `BenchActuationRelease`; describe the changed purpose. |
| `App/Inc/gp_diag.h`, `App/Src/gp_app.c` | Include timer clock in the register snapshot. |
| `App/Src/gp_diag_format.c` | Explicit armed/motor state, duty, STEP Hz and DIR. |
| `tests/host/test_diag.c` | Full duty/frequency, stopped-channel, direction and formatting tests. |
| `tests/firmware/test_canbench_release.py` | Resolve actual presets, compile assertions, optionally verify configured cache and built manifest/artifact hashes. |
| `tests/host/CMakeLists.txt` | Run the new preset regression in the normal suite. |
| `docs/CANBENCH_RELEASE_ACTUATION.md` | This report and hardware checklist. |
| `docs/CURRENT_BOARD.md`, `docs/CAN_LINK.md`, `docs/BUILDING.md` | Identify active CanBenchRelease and retained receive-only CanBench. |
| `docs/ACTUATION_DIAGNOSIS.md` | Mark earlier CanBenchRelease configuration/results as historical. |

Earlier audit edits and existing IDE changes are preserved. No reset, commit,
push or flash was performed.

## Verification and artifact

Executed `cmake --preset CanBenchRelease` and
`cmake --build --preset CanBenchRelease` using the installed STM32 tools.

- CanBenchRelease and CanBench configured, compiled, linked and generated HEX;
  their profile-specific linked-image validators passed.
- **134/134 host tests passed in Debug and 134/134 in Release**, including full
  commissioning range reachability, codec/safety paths, shutdown, timeout,
  reversal/setup, malformed/stale rejection, diagnostics and preset regression.
- The new regression checked actual inheritance/profile macros, CAN timing,
  full caps, retained slew, routes/timing, receive-only CanBench, and the built
  CanBenchRelease cache, feature manifest and matching ELF/HEX hashes.
- Strict enabled-code syntax, nine negative CMake cases, peripheral ownership
  and current CubeMX hook checks passed. CubeMX was not regenerated.
- The active ELF includes CAN HAL/filter callbacks, production codec,
  supervisor, mapper, driver, `GpMotorHw_Write`, `GpBsp_MotorWrite`,
  `HAL_TIM_PWM_Init` and `HAL_TIM_PWM_ConfigChannel`. The inhibited initializer
  remains linked as a startup-failure fallback; its presence is not an inhibit flag.
- CanBenchRelease reserves **24392 / 65536 flash bytes** and **13256 / 20480 RAM
  bytes**. Runtime timing and stack margins still need hardware verification.

Runner and logs: `build/canbench-release-actuation/verify.py` and adjacent
`*.log` files. Hardware initialization, signals and motion were not tested.

Exact HEX:

```text
D:\Farah's Embedded Repo\CUERT\EmbeddedSensingNode\Bluepill-Actuator-Node\build\CanBenchRelease\Bluepill-Actuator-Node.hex
```

SHA-256: `1e6c0ab1723fb0f133887892af414d5e4118e7a4a880f282f95ae37ab18e30f3`.
Adjacent artifacts include ELF, MAP, manifest, hashes and the identical
`CUERT-Bluepill-DualAxis-Commissioning.hex`. Manifest: CAN=true, actuation=true,
decoder=true, watchdog=false, physical verification=false.

## Hardware checklist

1. **Disconnect motor power; retain required logic power.** Check the new HEX
   and startup `PROFILE can=1 actuation=1 signal=0 bench=1 wd=0`. CanBenchRelease
   can now output full-range signals. USART remains PA9 TX / PA10 RX, 115200 8N1.
2. Confirm rising RX/decode counts and joystick values as before, CAN ready=1,
   motor=1, recent task timestamps, id=105/dlc=8 and no unexpected fault. Use
   the [detailed pipeline checklist](ACTUATION_DIAGNOSIS.md) if propagation fails.
3. While neutral, stream VALID with ARM low (flags=1 or 5), observe state=2,
   then ARM+DEADMAN (flags=7), still neutral, and observe state=3/armed=1.
   Use fresh sequences, for example every 20 ms. Resolve/recover any latched
   fault rather than bypassing permissions.
4. **Scope throttle with energy isolated.** Test intermediate and full demand;
   allow the 10-second ramp. PB3 has 20 kHz PWM at intermediate duties and DC
   high at 100%. Full-scale TIM2: PSC=0, ARR=3599, CCR2=3600, CEN/CC2E on;
   PB4 stays low. This is not a powered full-throttle instruction.
5. **Scope steering with energy isolated.** Check PA6 STEP and PA7 DIR for
   positive, neutral and negative demand; allow the 5-second ramp. Check high/low
   pulse widths, DIR setup and reversal stops at MCU pins and again after the
   level shifter at driver terminals. A full-rate waveform proves no mechanical
   capability. Appropriate measurement bandwidth is required.
6. Test independent neutral, STOP, ARM/DEADMAN loss, >=200 ms CAN loss, timeout
   latching, recovery/new handshake and reset while energy is isolated. Do not
   halt an energized system in a debugger: timers can continue while CPU control
   is stopped. STEP-off does not establish loss of holding torque.
7. **Before powered tests**, establish and configure the validated STEP limit.
   Check supply polarity/ratings, driver current/microstep settings, external
   enables and power settling, direction, electrical compatibility and travel
   limits. Provide an independent, accessible emergency power cutoff.
8. Only after those checks pass, secure the vehicle and test each axis with
   small reviewed demands, then both. Observe current, direction, motion and
   shutdown. Full normalized range is not permission for full speed/torque.
   Correct driver-terminal signals with no motion point to power, enable,
   driver alarms/configuration, motor wiring or mechanics.
