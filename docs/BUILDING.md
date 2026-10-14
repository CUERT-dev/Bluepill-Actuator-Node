# Building

## Tools

- Arm GNU Toolchain 14.2.Rel1 (arm-none-eabi), must match CI
- CMake 3.28 or newer
- Ninja 1.10 or newer
- cppcheck 2.x

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

Available once `tests/host/CMakeLists.txt` exists:

    cmake -S tests/host -B build/host -G Ninja
    cmake --build build/host
    ctest --test-dir build/host --output-on-failure

## Static analysis

    cppcheck --enable=warning,style,performance,portability --inline-suppr --quiet <source dirs>

CI publishes the cppcheck report as an artifact. It is report-only until the baseline is clean and the owners agree to make it a gate.

## CI

`.github/workflows/gp-ci.yml` runs on pull requests into Testing and Maintaining and on pushes to Testing, Maintaining and `ci/**`. Artifacts are labelled with the candidate SHA. CI never flashes hardware.

## Troubleshooting

- `cmake --preset Debug` fails with an unknown preset: run it from the repository root where `CMakePresets.json` lives.
- Toolchain not found: check `arm-none-eabi-gcc --version` and that its `bin` directory is on PATH.
- Do not commit machine-specific paths; keep them in `CMakeUserPresets.json`.
