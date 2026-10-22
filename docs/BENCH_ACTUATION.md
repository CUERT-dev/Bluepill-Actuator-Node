# Bench actuation

The real dual-axis commissioning image is now generated. See
[COMMISSIONING_CANDIDATE.md](COMMISSIONING_CANDIDATE.md) for the artifact,
parameters, build and flashing commands, preserved safeguards and first-session
checks. This replaces the earlier unapproved-template status of this document.

The previous exact build blockers were the two physical-approval `#error` gates
(`GP_BENCH_COMMISSIONING_APPROVED` and
`GP_STEERING_EXTERNAL_READY_BEFORE_ARM`) plus unset numeric settings in
`App/Profiles/bench_actuation.h`. Physical verification is recorded as false;
it is no longer a compilation prerequisite for an explicitly selected candidate.
`App/Profiles/commissioning_parameters.h` now provides documented provisional
settings. No physical input or readiness callback has been fabricated.

`BenchActuation`, `BenchActuationRelease` and `Commissioning` build the real
CAN/decoder/supervisor/mapper/driver/STM32 adapter application with actuation ON.
`Commissioning` selects Release optimization. Explicit mode selection, valid
CAN/decoder/settings, supported pin route, numeric bounds and component timing
checks remain mandatory. Watchdog approval gates remain; these profiles use
IWDG OFF. `SignalTest` and `SignalTestRelease` remain available only with motor
energy disconnected.

Runtime permission still requires successful timer/adapter/CAN initialization,
no CAN fault or backlog, valid fresh revision-10 commands, fresh neutral ARM-low
then an ARM edge with DEADMAN, no STOP/timeout/stale generation/latched fault,
and explicit recovery plus a new handshake after faults. Both motors operate
together. Each neutral axis stops independently, with the existing brief
all-output inhibition during steering direction setup/reversal retained.
Steering neutral requests no automatic centering.

Future independent E-stop, interlock, enable/readback or driver-readiness sensing
requires an actual hardware revision with defined wiring, voltage levels and
failure behavior. None exists on this board. Software waveform shutdown and
independent interruption of motor energy remain separate functions.
