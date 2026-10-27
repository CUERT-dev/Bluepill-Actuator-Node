# Steering vibration and latched 0x400: 27 October 2026

> Latest follow-up: [DIRECT_COMMANDS.md](DIRECT_COMMANDS.md) documents v4 and
> current artifact hashes. Direct mode now automatically resumes on a new fresh
> valid command after timeout, queue-age, backlog or timing shutdown. The
> v3 verification/hashes and manual-recovery behavior below are historical.

## What the supplied log establishes

CAN is receiving and decoding about 50 frames/s, with no reported rejection,
drop or CAN error. `fault=00000400`, `state=4`, `have=0` and the frozen
`cmd_ms=76800` show a latched GP_FAULT_BACKLOG shutdown. TIM2 and TIM3 have
their channels/counters disabled. The received nonzero demands cannot actuate
while this fault is latched. `motor=1` means initialized, not turning.

The earlier priority change did not cover all receive/control interleavings.
The old controller also latched 0x400 for a newly arrived hardware FIFO frame,
an in-flight decode, or a new event published while mapping outside the lock.
These ordinary arrivals are now covered by deterministic regression tests.
The log does not identify which backlog branch fired on the physical board.

The retained TIM3 PSC=0 / ARR=2086 describes a prior approximately 34,499 Hz
period, not an active signal: CCR=0 and CEN/CC1E=0 at the reported time.
This shutdown explains the later zero outputs, but does not prove the cause of
the vibration that preceded it.

## Firmware changes

- Ordinary receive races hold both outputs inactive and return GP_PENDING.
  Packets and their original timestamps remain queued, including possible STOP.
  Decoding can complete without a recovery command or new operator permission.
- Persistent receive waiting beyond the existing 40 ms stage budget still
  latches GP_FAULT_BACKLOG. Event-budget exhaustion, overflows, queue age,
  timeout, CAN errors and output faults retain shutdown behavior. No actual
  latched fault is automatically cleared.
- Mapping for an uncommitted output is discarded. Output history restarts at
  zero; prior fractional slew credit and an already-inactive driver's setup
  transaction survive, preventing low-rate steering startup starvation.
- `ACT rx_wait=1` identifies temporary receive inhibition; `rx_waits` counts
  episodes. Frequent episodes can interrupt motion and require timing analysis.
  The boot identifier is now `actuation-diag-v3` and reports STEP full scale.
- Direct valid commands remain enabled, with no ARM/DEADMAN/neutral handshake.

## Images

`build/CanBenchRelease/Bluepill-Actuator-Node.hex` contains the race fix with
the existing full-range settings: throttle 0..100%, steering full scale
166666 requested pulses/s (approximately 166282 realized at 72 MHz).

`build/SteeringBenchRelease/Bluepill-Actuator-Node.hex` is an additional low-speed
diagnostic. It changes only steering full scale to 1000 pulses/s, keeping the
full signed command range, direct policy, throttle, pins and electrical timing.
The 200 permille/s steering ramp now means 200 pulses/s per second. At 40000
pulses/revolution, full steering is only 1.5 RPM; a partial command moves very
slowly. This test setting is not a validated mechanical operating limit.

Both build with CMake configure/build presets of the corresponding name.
Each directory contains a manifest and SHA.txt identifying the exact artifacts.
These images were built locally; no board was flashed or mechanically tested.

## Reported hardware and remaining checks

The user identified an 86CM85 motor and EM882S driver, and clarified that PUL/DIR
pass through a level converter. Do not diagnose a direct STM32 3.3 V connection.
Taking down as ON and only switches 2, 3, 4 down gives SW1..8:
OFF, ON, ON, ON, OFF, OFF, OFF, OFF. The manual maps this to 2.7 A peak /
1.93 A RMS, full idle current, and 40000 pulses/revolution. The 86CM85 is rated
6 A phase current. Review the selected drive current against the motor/load;
the supplied log does not establish that low current caused the vibration.

At that microstep setting the prior 34,499 Hz period represents about 51.7 RPM.
The original electrical ceiling alone therefore does not establish overspeed.
Supply voltage, mechanical coupling/load, converter output waveform and the
drive's pulse-mode configuration remain unconfirmed.

Measure PUL+ relative to PUL- at the driver, including pulse amplitude, edges
and width, and DIR relative to DIR-. A converter's presence does not measure its
loaded output. With power disconnected, verify that A+/A- are one winding and
B+/B- the other. Check the driver's alarm indication. Never reconnect motor
leads under power. Confirm pulse/direction mode rather than CW/CCW mode.

Sources: [EM882S manual, sections 2, 3, 7, 10, 12](https://www.leadshine.com/upfiles/downloads/df11830f96faa1fe4df379db040bb17f_1650880565511.pdf)
and [Leadshine 86CM85 specifications](https://en.leadshine.com/product-detail/86CM85.html).

## Verification

Regression coverage includes raw and hardware receive races, in-flight decoding,
final-commit STOP and valid events, bounded persistent waits across clock wrap,
1000 actual codec frames at 50 Hz with motor-first dispatch, and real-driver
setup progress during repeated races at fractional slew. Both steering profiles
are exercised through codec, supervisor, mapper, driver and timer planning in
both directions, including neutral and STOP.

Final results: **141/141 host tests passed in Debug and Release**. All three
profiles (CanBenchRelease, SteeringBenchRelease, inhibited CanBench) built and
passed linked firmware checks. Actual preset/cache/manifest/hash checks,
enabled-code syntax, ten negative configuration cases, peripheral ownership
and CubeMX hook checks passed. Both HEX files also passed record-length,
checksum and EOF validation. Each active image uses 25112 bytes of flash and
13264 bytes of linker-reserved RAM.

```text
CanBenchRelease/Bluepill-Actuator-Node.hex
SHA256 00daa013993c3bab6a18b63209d01d241349903366f7aa57706ea1f03ae4d33d

SteeringBenchRelease/Bluepill-Actuator-Node.hex
SHA256 b362bda076a1671077dcf86fbdf8267d996500de5dd52fd593b7d3646e3a8f88
```
