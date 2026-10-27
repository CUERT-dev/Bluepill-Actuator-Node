> **Current commissioning artifact:** [COMMISSIONING_CANDIDATE.md](COMMISSIONING_CANDIDATE.md)
> provides the real dual-axis actuation-enabled HEX, build/flash instructions and
> provisional settings. All ten profiles link; physical verification is pending.
> This supersedes historical approval/template and motor-disabled status below.
>
> **Actuation gates:** [BENCH_ACTUATION.md](BENCH_ACTUATION.md) documents the
> bench profiles and the physical UNKNOWNs to check during commissioning.
>
> **Current board:** [CURRENT_BOARD.md](CURRENT_BOARD.md) supersedes earlier
> five-callback requirements and documents the usable unpowered SignalTest builds.
>
> **Independent audit:** See [INDEPENDENT_AUDIT.md](INDEPENDENT_AUDIT.md) for the
> two subsequent fixes, 116-case test results and current image readiness.
>
> **21 October maintainer update:** See [MAINTAINER_BRINGUP.md](MAINTAINER_BRINGUP.md)
> and [MAINTAINER_VALIDATION.md](MAINTAINER_VALIDATION.md). CAN 500 kbit/s bench
> profiles, UART diagnostics/recovery and inhibited-supervisor operation now exist.
> The earlier status below is retained as historical context. Motors and IWDG
> remain disabled pending real hardware prerequisites.

# Building

## Normal Release build (updated 27 October 2026)

`Release` now uses the existing full dual-axis commissioning configuration:
CAN enabled at 500 kbit/s on PB8/PB9, command decoding, motor supervision,
throttle PWM, steering STEP/DIR, and USART1 debug at 115200 8N1.

```powershell
cmake --preset Release
cmake --build --preset Release
```

Flash `build/Release/Bluepill-Actuator-Node.hex`. Unlike CanBench, this image
can drive motors after the existing neutral/ARM/deadman conditions are met.
It uses the existing provisional settings in `App/Profiles/commissioning_parameters.h`;
the build does not establish physical commissioning. The hardware watchdog
remains off because its reset-safe behavior and timing are not configured.
Debug/SafeFlash remain inhibited, and CanBench remains CAN-only. The older
default-build descriptions below are historical.

The current native FreeRTOS application, disabled hardware profile, artifacts
and remaining commissioning blockers are documented in
[FIRMWARE_INTEGRATION.md](FIRMWARE_INTEGRATION.md). Normal builds create ELF, HEX,
map, memory reports and SHA manifests. CAN, actuation and watchdog startup
default OFF. The production revision-10 CAN encoder/decoder is built in; no
operational hardware values or mock decoder are supplied. The final physical
pin map alone is confirmed; see [FINAL_SCHEMATIC_REVIEW.md](FINAL_SCHEMATIC_REVIEW.md).

## Tools

- Arm GNU Toolchain 14.2.Rel1 (arm-none-eabi), must match CI
- CMake 3.28 or newer
- Ninja 1.10 or newer
- cppcheck 2.x
- Python 3 (artifact metadata and configuration checks)

## Check versions

    arm-none-eabi-gcc --version
    cmake --version
    ninja --version
    cppcheck --version

## Build

From the repository root:

    cmake --preset Debug
    cmake --build --preset Debug

Outputs land in `build/`. The `.elf` is the flashable image; CI also publishes an Intel HEX, the map file and size output.

## Host tests

The mapping and driver tests compile the same `gp_actuator` target as the
firmware, using a native host C compiler and mock hardware callbacks:

    cmake -S tests/host -B build/host -G Ninja
    cmake --build build/host
    ctest --test-dir build/host --output-on-failure

The 18 deterministic cases cover mapping, validation, shutdown, reversal,
pending adapter setup, rollover and the combined pipeline. Both header include
orders are compiled. The existing CI host-tests job discovers this harness
automatically. Run host configuration without the ARM toolchain/preset.

For callback timing responsibilities and the integration validation results,
see [ACTUATOR_INTEGRATION.md](ACTUATOR_INTEGRATION.md).

The harness preserves the original 43 portable CAN/command/safety cases.
Those use a test-only decoder and deterministic interrupt/task interleavings;
the 6 new codec/supervisor cases pass the real `GP_DecodeCan` directly. All
use fake clocks and mock hardware callbacks. The firmware now includes the
native FreeRTOS port, real HAL backend and production CAN codec; approved
hardware configuration/board interlocks remain unavailable. See
[BLUEPILL_ARCHITECTURE.md](BLUEPILL_ARCHITECTURE.md) for integration boundaries,
shutdown ordering, memory bounds and the remaining hardware decisions.

The complete harness now has **102 host cases**: the prior 79, 7 codec cases,
6 real-decoder/supervisor cases, 9 neutral regressions and 1 additional
driver/adapter-policy case. The supplied codec test runs with assertions enabled.
Run the default firmware and compile-only gated branch checks with:

    cmake --preset Debug
    cmake --build --preset Debug
    cmake --build build/Debug --target gp_check_enabled_syntax
    cmake --preset Release
    cmake --build --preset Release

`gp_check_enabled_syntax` does not link an enabled image. The production codec
is always registered as `gp_can_codec`. Obsolete `GP_DECODER_SOURCES` and
`GP_DECODER_INCLUDE_DIRS` settings must be removed; duplicate providers are
rejected. Future reviewed board providers use `GP_BOARD_APPROVAL_HEADER` and
`GP_BOARD_SOURCES`. Hardware options are
`GP_ENABLE_CAN`, `GP_ENABLE_ACTUATION` and `GP_ENABLE_WATCHDOG`; enabling them
without their dependencies/approvals fails configuration or compilation.

## Static analysis

    cppcheck --enable=warning,style,performance,portability --inline-suppr --quiet <source dirs>

CI publishes the cppcheck report as an artifact. It is report-only until the baseline is clean and the owners agree to make it a gate.

## CI

`.github/workflows/gp-ci.yml` runs on pull requests into Testing and Maintaining and on pushes to Testing, Maintaining and `ci/**`. Artifacts are labelled with the candidate SHA. CI never flashes hardware.

## Troubleshooting

- `cmake --preset Debug` fails with an unknown preset: run it from the repository root where `CMakePresets.json` lives.
- Toolchain not found: check `arm-none-eabi-gcc --version` and that its `bin` directory is on PATH.
- Do not commit machine-specific paths; keep them in `CMakeUserPresets.json`.
