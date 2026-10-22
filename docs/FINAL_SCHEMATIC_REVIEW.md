> **21 October maintainer update:** See [MAINTAINER_BRINGUP.md](MAINTAINER_BRINGUP.md)
> and [MAINTAINER_VALIDATION.md](MAINTAINER_VALIDATION.md). CAN 500 kbit/s bench
> profiles, UART diagnostics/recovery and inhibited-supervisor operation now exist.
> The earlier status below is retained as historical context. Motors and IWDG
> remain disabled pending real hardware prerequisites.

**Final schematic alignment review — 20 October 2026**

The user-supplied [final pinout](hardware/final_pinout.png) replaces every earlier
provisional wiring reference. Its SHA-256 is
`6a2a47bbea55377739ba0c5e9950f6c5519f240f6f90344df84b2d60efa22a52`.
This review covers the Bluepill actuator node, not the other sensing board drawn
on the sheet. The screenshot establishes intended connections; it is not a PCB
netlist, continuity test, component specification or electrical approval.

**Wiring and peripheral compatibility.**

| Final connection | Actual firmware / required peripheral mode | Status |
| --- | --- | --- |
| PB8 CAN RX / PB9 CAN TX | `GpBsp_CanStart`: `__HAL_AFIO_REMAP_CAN1_2`, PB8 input, PB9 AF push-pull | Correct gated route; timing still unapproved |
| PB3 BTS7960 L-PWM | TIM2_CH2, `__HAL_AFIO_REMAP_TIM2_PARTIAL_1` | Correct physical route; L-PWM forward is **unknown** |
| PB4 BTS7960 R-PWM | GPIO held at the separately approved inactive level in an actuation profile | Correct for the existing single PWM adapter; no PB4 PWM implementation |
| PA6 EM882S PUL+ through level shifter | TIM3_CH1, `__HAL_AFIO_REMAP_TIM3_DISABLE` | Correct; no software STEP toggling |
| PA7 EM882S DIR+ through level shifter | GPIO push-pull only after permission/setup | Correct; DIR polarity remains unknown |
| PA9 USART1 TX / PA10 USART1 RX | USART1 default pin map; no UART initialization, task, ISR or application transport | Pins available; firmware leaves them unused |
| PA13 SWDIO / PA14 SWCLK | Serial Wire `.ioc`, `HAL_MspInit` and BSP `SWJ_NOJTAG` | SWD retained; JTAG disabled; verify external debug-header wiring |

The TIM2, TIM3, CAN and SWJ remap fields are independent. TIM2 partial remap 1
and TIM3 without remap therefore coexist with PB8/PB9 CAN and PA13/PA14 SWD.
PA7's optional TIM3_CH2 function does not conflict with GPIO DIR: channel 2 is
not configured/enabled and PA7 is not connected to an AF output. Unused TIM2
channels do not drive their possible pins. USART1 remains on its default route.
Do not enable SPI1, remapped I2C1, TIM4 outputs or other functions on owned pins.
PB3 is also JTDO/TRACESWO: do not enable JTAG or SWO trace on this motor net.

PB4 can obtain TIM3_CH1 through **TIM3 partial remap**, but that would move STEP
away from PA6. It is not a second independent channel on the final arrangement.
If motor testing establishes that R-PWM is the required forward input, the
existing route must remain inhibited until a compatible hardware/timer solution
is reviewed. No software PWM or silent remap was added.

These mappings were checked against ST's
[STM32F103x8/xB datasheet, Table 5](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
and [RM0008 GPIO/AFIO remapping and debug configuration](https://www.st.com.cn/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
as well as the installed genuine HAL/CMSIS headers. In particular, SWJ_CFG is
write-only/undefined on read: the installed HAL remap helpers account for it;
no speculative register-readback test or vendor modification was introduced.

**GPIO, reset and shutdown audit.**

`GpBsp_Init` enables the GPIO/timer clocks and disables timer channels/counters.
In the default profile, PB3/PB4 and PA6/PA7 become analog/high impedance; no PWM
or STEP AF route is selected. CAN does not initialize PB8/PB9, and USART1 does
not initialize PA9/PA10. GPIO writes in an approved actuation profile establish
the selected inactive latch before switching the pin to an output. Running
timer updates retain the existing preload policy; no adapter was rewritten.

Application analog configuration applies **after startup**, not during reset.
RM0008 describes floating GPIO reset defaults and debug exceptions including
PB4/NJTRST's pull-up. PB3 belongs to the debug port until JTAG is released.
The earliest HAL MSP releases JTAG while preserving SWD, followed by BSP pin
configuration. Neither sequence controls the driver while reset is asserted,
before firmware runs, in a bootloader, or if a debugger reclaims a motor pin.
Unknown external pulls and level-shifter defaults cannot be made safe by naming
an analog mode. Verify BOOT straps, power sequencing and actual reset waveforms.

The schematic shows BTS7960 R-EN/L-EN connected to the +3.3 V rail, with no MCU
enable net. EM882S ENA+/ENA- and alarm terminals appear unconnected. It shows no
independent physical E-stop or MCU safety readback. Consequently:

- PWM-off can request zero drive but cannot disconnect motor energy. Braking,
  coasting and behavior with both PWM inputs inactive require the actual module
  truth table and powered measurements. Enables tied high cannot be disabled by
  a callback or by resetting the MCU.
- STEP-off stops requested stepping, not necessarily winding current or holding
  torque. Unconnected ENA gives firmware no disable control. Steering neutral
  does not center wheels or release holding torque.
- Fatal faults can halt software while peripherals retain state. The existing
  fatal handler does not claim a damaged kernel can safely execute motor calls.
  An MCU watchdog reset restores MCU reset states; it does not cut driver power.

**A. Fully completed Bluepill software.**

The production codec, ordered RX/safety queues, generation barriers, independent
neutral policy, mapping/driver, static FreeRTOS tasks, exception/CAN interrupts,
HAL backend, timer adapters and watchdog supervision infrastructure remain in
place. No completed control module was redesigned. The default build still
sets CAN startup, actuation and watchdog OFF.

This review records `GP_PIN_MAP_APPROVED=1` for **physical assignment only**.
All electrical approval flags, unknown polarities, zero timing/limit placeholders
and unselected `GP_THROTTLE_FORWARD_ROUTE=0` remain. Existing build/compile gates
still prevent operational startup. The board-port declarations now live in
`App/Inc/gp_board_port.h`; there is no supplied implementation or healthy stub.

One missing hook was fixed: CubeMX's `assert_failed` now calls the existing fatal
handler when `USE_FULL_ASSERT` is enabled. The enabled syntax check covers that
branch and the regeneration checker checks its preservation. `GpApp_Create` is
declared non-returning, matching scheduler startup followed by fatal handling
on failure; this also removes a false reachable-fallback-loop analyzer report.

**B. Remaining software work independent of electrical approval.**

No further mandatory implementation was identified within this review that can
be completed without choosing an application interface or a hardware contract.
`GpApp_RequestRecovery` already supplies the task-context acknowledgement hook;
there is no selected caller in this schematic/application. Once a local service
or operator interface is specified, connect its acknowledgement to that hook.
The screenshot's serial connector alone does not define baud rate, framing,
authorization or a recovery command. No button pin, UART protocol, automatic
recovery or CAN recovery command was invented. Sender/gateway transient-invalid
STOP behavior still needs confirmation; the codec dependency itself is resolved.

**C. Software configuration dependent on approved hardware settings.**

- Provide a real `GP_BOARD_APPROVAL_HEADER`: CAN bitrate/prescaler/segments/SJW;
  PWM input selection/frequency/cap/slew; steering cap/slew, STEP frequency and
  high/low minima; DIR/ENA/setup/reversal timing; all active/inactive polarities.
- Supply `GP_BOARD_SOURCES` implementing the declared real, bounded safety and
  inhibit/readback callbacks after the actual circuit is defined. Tied-high
  enables and absent E-stop/ENA wiring do not justify unconditional healthy
  callbacks, a no-op inhibit or `GP_INDEPENDENT_ESTOP_APPROVED=1`.
- IWDG startup still needs reset-safe hardware approval, prescaler/reload, task
  deadline and maximum LSI frequency. It is intentionally irreversible once
  started until reset. Verify actual option bytes too: hardware-start watchdog
  mode cannot be disabled by the application's OFF option.
- The `.ioc` currently configures the clock/SWD foundation, **not** operational
  CAN/TIM/USART peripherals. Keep the existing gated BSP as the single owner.
  After timing/route approval, generate/reconcile CAN on PB8/PB9, TIM2_CH2 on
  PB3, TIM3_CH1 on PA6 and GPIO PA7/PB4. Add USART1 only if an application use is
  specified. Avoid duplicate generated init/IRQ functions or a second CMSIS
  RTOS startup. Preserve native RTOS exception hooks and run vector/hook checks.

**D. Physical hardware changes or tests outside firmware scope.**

Hardware owners must establish independent E-stop/energy isolation, default-off
behavior during MCU reset/debug/power loss, any required enable/readback circuit,
and an explicit holding-torque policy. The shown wiring cannot meet the existing
actuation prerequisites through software configuration alone. Verify the actual
BTS module logic supply/thresholds, transceiver supply/levels/termination, power
rails, level-shifter supply/topology/speed and EM882S input-current requirements.
Then measure inactive levels, PWM/STEP/DIR waveforms, setup/deadtime, cutoff
latency, fault/recovery behavior, stack/ISR load, and watchdog stall/reset
behavior, initially with motors disconnected. No such target/electrical tests
were performed here, and this review is not powered-operation approval.

**Revalidation and exact changes.**

All **102/102 existing host tests passed**, 0 failed. Debug and Release ARM GCC
14.3.1 builds passed, including the linked-symbol/vector/inhibited-profile checks.
Flash/RAM remain Debug 21,808/7,336 bytes and Release 13,108/7,192 bytes. Enabled
hardware/full-assert branches are checked without linking an enabled image.
Configuration gates, GCC analyzer and isolated CubeMX hook regeneration are
recorded with logs under `build/schematic-review-*`. `cppcheck` is unavailable
on PATH; the exact CI compiler/remote CI and target runtime were not exercised.

See [SCHEMATIC_REVIEW_CHANGED_FILES.txt](SCHEMATIC_REVIEW_CHANGED_FILES.txt) for
the exact files changed in this focused step. The canonical shared types, codec,
mapper, driver, supervisor, task logic and vendor sources remain unchanged.
Existing staged work was preserved. No flash, commit, push or merge occurred.
