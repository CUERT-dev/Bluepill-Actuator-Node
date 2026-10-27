# CanBenchRelease direct-command firmware

> Current build: `actuation-diag-v5`. [CAN_ZERO_INPUT.md](CAN_ZERO_INPUT.md)
> describes the additional raw CAN input report. Fresh valid commands automatically resume
> after transient stream/timing stops. [STEERING_DIAGNOSIS.md](STEERING_DIAGNOSIS.md)
> records the earlier receive-race investigation and motor settings.

27 October 2026, explicitly requested by the user. This supersedes the earlier
CanBenchRelease neutral/ARM/DEADMAN instructions.

**CanBenchRelease now acts on a valid fresh command without any operator ARM,
DEADMAN or neutral-before-arming sequence. A nonzero first command can cause
output after startup.** This removes operator authorization checks, not command
validation, fault protection or electrical timing.

## Behavior

- `GP_DIRECT_COMMANDS=ON` in CanBenchRelease and SteeringBenchRelease. Other presets default OFF;
  CanBench remains receive-only. The normal mapper API and normal supervisor
  arming behavior are preserved for other configurations.
- Fresh valid commands (including flags=0x01 with ARM/DEADMAN both absent) enter
  the active state directly. Received flags are not rewritten or fabricated.
  The next output interval applies the existing slew-limited demand. No neutral
  prerequisite, button edge or deadman hold is required.
- Throttle remains 0..1000 and steering -1000..1000 permille. Slew remains
  100 permille/s throttle and 200 permille/s steering: approximately 10 seconds
  and 5 seconds from zero to full output. Direction-change setup remains.
- STOP (canonical flags=0, neutral packet) immediately stops both outputs.
  Following STOP, a new valid command can resume output without a handshake.
- The 200 ms timeout, excessive queue age, receive backlog and update-timing
  violations stop outputs and discard buffered commands. In direct mode only,
  the next newly captured, decoded, fresh valid command automatically resumes
  output from zero. No serial `recover` command or operator handshake is needed.
  Fault history and the last stop reason remain available for diagnosis.
- Invalid CRC/ranges/flags and duplicate sequence cannot authorize output.
  Driver, dependency, E-stop, bus-off and overflow faults still require resolving
  the cause and explicit recovery. Mixed hardware/transient faults stay latched.
- CAN timing, ID, layout, decoder, pin routes and UART transport are unchanged.

The serial banner now says `DIRECT COMMANDS`. Each `ACT` line includes
`direct=1`. Once a fresh valid command is accepted, state=3 / armed=1 indicates
the active supervisor state; it does not imply that an ARM switch was pressed.
Check `motor=1`, `fault=00000000`, requested/applied outputs and PWM/STEP fields.
`pending=1` can briefly hold both outputs off during steering setup.
`received=throttle,steering` on the ACT line always reports the last decoded CAN
values, independently of a shutdown-cleared `requested` or inactive `applied`.

## Implementation

`CMakePresets.json` selects the policy, CMake passes it to the app and the
firmware manifest records `direct_commands=true`. `gp_board_config.h` defaults
to normal arming and rejects direct mode without CAN/output support.
`GpControlConfig.direct_commands` is set by `GpApp_Create`. The supervisor
accepts valid demand directly in this mode and calls the explicit
`CommandMap_UpdateDirect` entry point. The mapper still checks validity,
numerical ranges, update timing and supervisor enable; normal
`CommandMap_Update` continues to require ARM/DEADMAN. The output driver and
register-only shutdown implementation are unchanged.

Regression tests cover nonzero startup without any operator flags, operator
flag changes, full-range throttle and both steering signs, independent neutral,
STOP/resume, duplicate/malformed rejection, automatic transient-stop resumption,
expired restart-event rejection, and latched hardware/overflow faults. Both
profiles exercise steering-only commands with zero throttle and VALID alone,
then timeout and restart in the other direction through the real driver and
timer planner. Existing normal-arming tests remain in the suite.

## Verification and exact artifact

- CanBenchRelease, SteeringBenchRelease and CanBench configure/build/link/HEX generation passed.
- **142/142 host tests passed in Debug and 142/142 in Release.**
- Actual preset/cache/manifest/ELF/HEX regression passed, including direct mode.
- Enabled-code syntax checks cover normal and direct modes; all ten negative
  CMake cases, peripheral ownership and current CubeMX hook checks passed.
- Flash usage: 25856/65536 bytes; linker-reserved RAM: 13280/20480 bytes.
- HEX checksums, record lengths and EOF validated for both active profiles.
- No physical board, power stage or actuator was tested or flashed.

```text
D:\Farah's Embedded Repo\CUERT\EmbeddedSensingNode\Bluepill-Actuator-Node\build\CanBenchRelease\Bluepill-Actuator-Node.hex
SHA256=b76c84faba48495a10493f4b011da53ff9396fa545c1256070b5dd9b92649b39

build/SteeringBenchRelease/Bluepill-Actuator-Node.hex
SHA256=9a4800b16c2002e4590739bdeaeaee0a1fc0e4e483402a54c74aaecd201f2414
```

The adjacent commissioning-named HEX is an identical copy. Build logs and
reproduction runner remain under `build/canbench-release-actuation/`.

Start waveform checks with motor power disconnected. This board has no
MCU-connected motor enable or emergency-stop readback, and this change does not
create physical power isolation. The retained steering waveform maximum is
approximately 166282 pulses/s, **not a validated installed mechanical speed**.
Validated steering limits, driver settings, direction, electrical compatibility,
secured mechanics and independent power cutoff remain prerequisites for powered
testing. Enabling direct mode makes any valid nonzero startup demand actionable.
