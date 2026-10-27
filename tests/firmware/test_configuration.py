"""Exercise actual CMake gates in isolated build directories; never flash."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--cmake', default='cmake')
p.add_argument('--root', default='.')
p.add_argument('--output', default='build/configuration-checks')
a = p.parse_args()
root = Path(a.root).resolve()
output = Path(a.output).resolve()
output.mkdir(parents=True, exist_ok=True)
cases = {
    'direct_without_outputs': (['-DGP_DIRECT_COMMANDS=ON'], 'Direct commands require CAN and actuation enabled'),
    'missing_approval': (['-DGP_ENABLE_CAN=ON'], 'requires GP_BOARD_APPROVAL_HEADER'),
    'duplicate_decoder': (['-DGP_DECODER_SOURCES=not-a-provider.c'], 'production CAN codec is built in'),
    'test_approval': (['-DGP_BOARD_APPROVAL_HEADER=' + str(root / 'tests/fixtures/compile_only_board.h')], 'Test fixtures'),
    'powered_denied': (['-DGP_ENABLE_CAN=ON', '-DGP_ENABLE_ACTUATION=ON',
                     '-DGP_BOARD_APPROVAL_HEADER=' + str(root / 'App/Profiles/can_500k.h')], 'Powered outputs require an explicit controlled bench'),
    'bench_missing_can': (['-DGP_CONTROLLED_BENCH_ACTUATION=ON', '-DGP_ENABLE_ACTUATION=ON'],
                          'Controlled bench actuation requires CAN'),
    'both_modes': (['-DGP_UNPOWERED_SIGNAL_TEST=ON', '-DGP_CONTROLLED_BENCH_ACTUATION=ON'],
                    'Select exactly one output mode'),
    'signal_missing_outputs': (['-DGP_UNPOWERED_SIGNAL_TEST=ON'], 'requires CAN and waveform outputs'),
    'signal_watchdog': (['-DGP_UNPOWERED_SIGNAL_TEST=ON', '-DGP_ENABLE_ACTUATION=ON',
                        '-DGP_ENABLE_CAN=ON', '-DGP_ENABLE_WATCHDOG=ON'], 'requires CAN and waveform outputs'),
    'obsolete_provider': (['-DGP_BOARD_SOURCES=unused-board.c'], 'not supported by the current board'),
}
for name, (flags, expected) in cases.items():
    result = subprocess.run([a.cmake, '-S', str(root), '-B', str(output / name), '-G', 'Ninja',
                             '-DCMAKE_TOOLCHAIN_FILE=' + str(root / 'cmake/gcc-arm-none-eabi.cmake'),
                             *flags], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output / (name + '.log')).write_text(result.stdout)
    if result.returncode == 0 or expected not in result.stdout:
        raise SystemExit('FAIL ' + name + '\n' + result.stdout)
    print('PASS', name)
