"""Guard actual preset inheritance/profile macros and optional built artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--root', required=True)
p.add_argument('--cc', required=True)
p.add_argument('--build-dir')
p.add_argument('--preset', default='CanBenchRelease',
               choices=('CanBenchRelease', 'SteeringBenchRelease'))
a = p.parse_args()
root = Path(a.root).resolve()
presets = json.loads((root / 'CMakePresets.json').read_text())
by_name = {item['name']: item for item in presets['configurePresets']}


def resolve(name, visiting=()):
    if name in visiting:
        raise SystemExit('cyclic preset inheritance: ' + name)
    item = by_name[name]
    parents = item.get('inherits', [])
    if isinstance(parents, str):
        parents = [parents]
    result = {}
    # CMake gives the first parent precedence when several specify a variable.
    for parent in reversed(parents):
        result.update(resolve(parent, (*visiting, name)))
    result.update(item.get('cacheVariables', {}))
    return result


enabled = resolve('CanBenchRelease')
expected = resolve('BenchActuationRelease')
expected['GP_DIRECT_COMMANDS'] = 'ON'
if enabled != expected:
    raise SystemExit('CanBenchRelease must reuse commissioning settings with direct commands ON')
checks = '''
#include "gp_board_config.h"
_Static_assert(GP_ENABLE_CAN && GP_HAS_DECODER, "real CAN and decoder required");
_Static_assert(GP_CAN_BITRATE == 500000 && GP_CAN_PRESCALER == 4 &&
 GP_CAN_BS1_TQ == 15 && GP_CAN_BS2_TQ == 2 && GP_CAN_SJW_TQ == 1,
 "preserve tested CAN timing");
_Static_assert(!GP_ENABLE_WATCHDOG && !GP_UNPOWERED_SIGNAL_TEST,
 "preserve existing watchdog policy and profile mode");
'''
for name, actuation in (('CanBenchRelease', True), ('CanBench', False), ('SteeringBenchRelease', True)):
    cfg = resolve(name)
    header = cfg['GP_BOARD_APPROVAL_HEADER'].replace('${sourceDir}', root.as_posix())
    command = [a.cc, '-std=c11', '-x', 'c', '-fsyntax-only',
               '-I' + str(root / 'App/Inc')]
    for flag in ('GP_ENABLE_CAN', 'GP_ENABLE_ACTUATION', 'GP_ENABLE_WATCHDOG',
                 'GP_UNPOWERED_SIGNAL_TEST', 'GP_CONTROLLED_BENCH_ACTUATION', 'GP_DIRECT_COMMANDS'):
        command.append('-D' + flag + '=' + ('1' if cfg[flag] == 'ON' else '0'))
    command.append('-DGP_BOARD_APPROVAL_HEADER="' + header + '"')
    source = checks
    if actuation:
        source += '''
_Static_assert(GP_ENABLE_ACTUATION && GP_CONTROLLED_BENCH_ACTUATION,
 "CanBenchRelease must initialize real output drivers");
_Static_assert(GP_DIRECT_COMMANDS, "CanBenchRelease must not require operator arming");
_Static_assert(GP_THROTTLE_CAP_PM == 1000 && GP_STEERING_CAP_PM == 1000,
 "full normalized range, no arbitrary amplitude caps");
_Static_assert(GP_THROTTLE_SLEW_PM_S == 100 && GP_STEERING_SLEW_PM_S == 200,
 "retain the existing slew limits");
_Static_assert(GP_THROTTLE_FORWARD_ROUTE == 1 && GP_THROTTLE_PWM_HZ == 20000,
 "retain PB3 TIM2_CH2 route and PWM frequency");
_Static_assert(GP_STEP_PULSE_US >= 3 && GP_STEP_LOW_US >= 3 && GP_DIR_SETUP_US >= 5,
 "retain electrical timing checks");
'''
    else:
        source += '_Static_assert(!GP_ENABLE_ACTUATION && !GP_CONTROLLED_BENCH_ACTUATION && !GP_DIRECT_COMMANDS, "CanBench stays receive-only");\n'
    if actuation:
        hz = 1000 if name == 'SteeringBenchRelease' else 166666
        source += f'_Static_assert(GP_MAX_STEP_HZ == {hz}, "profile STEP full scale");\n'
    result = subprocess.run(command + ['-'], input=source, text=True, capture_output=True)
    if result.returncode:
        raise SystemExit(name + '\n' + result.stderr)
    print('PASS', name, 'effective preset and compiled profile')

if a.build_dir:
    folder = Path(a.build_dir).resolve()
    cache = {}
    for line in (folder / 'CMakeCache.txt').read_text().splitlines():
        if line and not line.startswith(('#', '//')) and '=' in line:
            key, value = line.split('=', 1)
            cache[key.split(':', 1)[0]] = value
    for key, value in resolve(a.preset).items():
        expected = str(value).replace('${sourceDir}', root.as_posix())
        actual = cache.get(key, '')
        if key == 'GP_BOARD_APPROVAL_HEADER':
            if Path(actual).resolve() == Path(expected).resolve():
                continue
        if actual != expected:
            raise SystemExit('configured build mismatch: ' + key)
    manifest = json.loads((folder / 'firmware-manifest.json').read_text())
    if manifest['features'] != dict(can=True, actuation=True, watchdog=False, decoder=True):
        raise SystemExit('built image is not CAN + actuation')
    if not manifest['direct_commands']:
        raise SystemExit('built image still requires operator arming')
    if (manifest['build_type'] != 'Release' or not manifest['controlled_bench_actuation']
            or manifest['unpowered_signal_test'] or manifest['physically_verified']):
        raise SystemExit('incorrect build mode or fabricated physical verification')
    for suffix in ('elf', 'hex'):
        name = 'Bluepill-Actuator-Node.' + suffix
        if hashlib.sha256((folder / name).read_bytes()).hexdigest() != manifest['artifacts'][name]['sha256']:
            raise SystemExit('artifact hash mismatch: ' + name)
    print('PASS configured', a.preset, 'cache, enabled manifest and ELF/HEX hashes')
