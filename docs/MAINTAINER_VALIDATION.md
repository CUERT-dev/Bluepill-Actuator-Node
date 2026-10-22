# Maintainer validation — 21 October 2026

> Historical pre-audit results. [INDEPENDENT_AUDIT.md](INDEPENDENT_AUDIT.md)
> supersedes the counts, image sizes and no-defect conclusion below: two defects
> were corrected and 116 host cases now pass. Use the independent-audit artifacts.

Implemented, compiled and host-tested; **not target-tested or physically verified**.
See [MAINTAINER_BRINGUP.md](MAINTAINER_BRINGUP.md) for the operating instructions,
callback blockers, component-source references and electrical preflight.

- Host CTest: **110/110 passed, zero failed**. All 102 existing cases retained.
  Eight additions cover the bounded UART parser, production-encoder CLI vector,
  inhibited supervisor, diagnostic freshness counters, cancellable long power
  readiness and three profile/component configuration gates.
- Existing regressions exercise decoder-to-supervisor, independent neutrals,
  STOP/deadman/timeout, sequence rejection, fault barriers/ARM recovery, bus-off/
  overflow, driver errors, setup/reversal/cancellation, timer arithmetic and
  watchdog progress. These host fixtures are not physical hardware tests.
- Five actual ARM builds passed. Both CAN bench images link real HAL CAN init,
  filter/start/notifications and the unique strong production codec symbols.
  ELF checks verify kernel/control symbols, exception and CAN/USART1 vectors,
  stack-preserving SVC/PendSV forwarding, and absence of the active motor adapter
  in all five profiles. Actuation/IWDG are OFF in every delivered image.
- Enabled motor/watchdog/full-assert branch syntax passes strict warnings with
  synthetic compile-only settings. No mocked or enabled motor firmware was linked.
- GCC `-fanalyzer -Wall -Wextra -Werror -Wpedantic`: ten translation units passed,
  zero diagnostics, including enabled BSP/application, UART and service parser.
- Four actual negative CMake configurations rejected absent approval, external
  duplicate codec, test approval header and absent board provider.
- Real isolated CubeMX quiet generation completed successfully. All original
  hooks plus USART1 forwarding survived. Exclusive peripheral/scheduler ownership
  and clock/SWD checks passed in both actual and regenerated sources.
- `git diff --check` passed. All tracked Drivers files equal index bytes and all
  36 imported vendor hashes match. Existing index SHA-256 is unchanged.
- Local tools: ARM GCC 14.3.1, native GCC 8.1, CMake 4.3.1, Ninja, Python 3.13,
  installed CubeMX and STM32Cube F1 V1.8.7. cppcheck is not installed locally;
  remote CI and the exact CI ARM 14.2.Rel1 compiler have not been run. CI is now
  configured to build all five profiles when the work is committed for review.

| Profile | Flash bytes / 65,536 | RAM reserved / 20,480 | CAN |
|---|---:|---:|---|
| Debug | 28,260 | 12,464 | OFF |
| Release | 18,276 | 12,320 | OFF |
| SafeFlash | 28,260 | 12,464 | OFF |
| CanBench | 30,644 | 12,464 | ON |
| CanBenchRelease | 19,588 | 12,320 | ON |

RAM includes static stacks and linker reservations; it is not a stack high-water
measurement. Source fingerprint shared by these images:

`4f898adac478855b24f655e8bb109330cf49eaca53457c10dafa0f11af651fff`

HEAD remains `b77a3c24822c4722dc4f9ab325edbb86ea09073c` on
`feature/gp-actuator-integration`; HEAD alone does not describe these uncommitted
sources. Each image's manifest records its own hashes and feature gates.

Artifacts are in `build/{Debug,Release,SafeFlash,CanBench,CanBenchRelease}/`:
`Bluepill-Actuator-Node.elf`, `.hex`, `.map`, `firmware-manifest.json`,
`firmware.size.txt`, `SHA.txt`. The new review bundle is
`build/maintainer-artifacts.zip`, with `build/maintainer-artifacts.zip.sha256`.
Older schematic-review bundles are historical, not the current firmware.

Logs are `build/maintainer-final-checks.log`, `build/maintainer-final-artifacts.log`,
`build/maintainer-cubemx.log`, `build/maintainer-analysis/gcc-analyzer.log`,
`build/configuration-checks/*.log`, and `build/host/Testing/Temporary/LastTest.log`.
The bundle includes these and a source snapshot of this task's non-vendor changes;
it is not a replacement for the complete repository and imported dependencies.

**Readiness:** A is ready for first-flash bench validation with motor energy
isolated. B is ready for real 500 kbit/s CAN bench validation under the same
condition. Neither is already physically validated. B decodes real frames but
cannot arm, publish control movement or recover past missing safety dependencies.
C's runtime path and asynchronous readiness interface are implemented; its
production image remains unbuildable without real board-port functions and
approved motor/reset settings. No independent software implementation blocker
was identified beyond those hardware-dependent providers/configuration. Target
measurements, wiring changes, component setup and commissioning remain mandatory.

**PR:** software is ready for review against `Maintaining`, with hardware blockers
explicit. Nothing was staged, committed, pushed, merged or flashed. Existing
user-staged IDE metadata was preserved and still needs the maintainer's inclusion
choice; do not bulk-stage the workspace blindly. All generated results are under
ignored `build/`, and no tracked build output was added.

In this workspace `git status` still reports cached vendor modification entries,
while `git diff -- Drivers` is empty and raw Git blob hashing confirms every
tracked vendor file equals the index. The index was left untouched; a later
normal Git index/stat refresh can update those display entries without staging
content changes. The restored files contribute no vendor patch to the PR.
