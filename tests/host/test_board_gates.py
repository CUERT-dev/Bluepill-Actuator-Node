"""Compile-only synthetic settings; never link an enabled firmware image."""
import argparse
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--cc', required=True)
parser.add_argument('--root', required=True)
parser.add_argument('--case', required=True)
args = parser.parse_args()
root = Path(args.root).resolve()
profile = {
    'GP_ENABLE_CAN': 1, 'GP_PIN_MAP_APPROVED': 1, 'GP_CAN_SETTINGS_APPROVED': 1,
    'GP_CAN_BITRATE': 500000, 'GP_CAN_PRESCALER': 4, 'GP_CAN_BS1_TQ': 13,
    'GP_CAN_BS2_TQ': 4, 'GP_CAN_SJW_TQ': 1, 'GP_ENABLE_ACTUATION': 1,
    'GP_HAS_DECODER': 1, 'GP_OUTPUT_SETTINGS_CONFIGURED': 1, 'GP_UNPOWERED_SIGNAL_TEST': 1,
    'GP_RESET_SAFE_APPROVED': 1, 'GP_THROTTLE_FORWARD_ROUTE': 1,
    'GP_PWM_ACTIVE_HIGH': 1, 'GP_STEP_ACTIVE_HIGH': 1, 'GP_DIR_POSITIVE_HIGH': 1,
    'GP_REVERSE_INACTIVE_HIGH': 0, 'GP_THROTTLE_PWM_HZ': 20000,
    'GP_THROTTLE_CAP_PM': 600, 'GP_STEERING_CAP_PM': 500,
    'GP_THROTTLE_SLEW_PM_S': 1000, 'GP_STEERING_SLEW_PM_S': 1000,
    'GP_MAX_STEP_HZ': 20000, 'GP_STEP_PULSE_US': 5, 'GP_STEP_LOW_US': 5,
    'GP_DIR_SETUP_US': 5, 'GP_ENABLE_SETUP_US': 0, 'GP_REVERSAL_DEADTIME_MS': 10,
}
case = args.case
expected = ''
if case == 'default':
    profile = {}
elif case == 'synthetic':
    pass
elif case == 'powered_denied':
    profile['GP_UNPOWERED_SIGNAL_TEST'] = 0
    expected = 'Powered outputs require an explicit controlled bench commissioning mode'
elif case.startswith('controlled_bench'):
    profile.update({'GP_UNPOWERED_SIGNAL_TEST': 0, 'GP_CONTROLLED_BENCH_ACTUATION': 1,
                    'GP_BENCH_COMMISSIONING_APPROVED': 1,
                    'GP_STEERING_EXTERNAL_READY_BEFORE_ARM': 1})
    if case == 'controlled_bench':
        pass  # Synthetic compile-only settings; never a firmware provider.
    elif case == 'controlled_bench_unapproved':
        profile['GP_BENCH_COMMISSIONING_APPROVED'] = 0
        # Commissioning candidates must compile without claiming approval.
    elif case == 'controlled_bench_no_readiness':
        profile['GP_STEERING_EXTERNAL_READY_BEFORE_ARM'] = 0
        # No electrical readiness readback or fictitious success is required.
    elif case == 'controlled_bench_both_modes':
        profile['GP_UNPOWERED_SIGNAL_TEST'] = 1
        expected = 'Select exactly one output mode'
    elif case == 'controlled_bench_template':
        profile = {'GP_ENABLE_CAN': 1, 'GP_ENABLE_ACTUATION': 1,
                   'GP_CONTROLLED_BENCH_ACTUATION': 1}
    elif case == 'controlled_bench_zero_cap':
        profile['GP_THROTTLE_CAP_PM'] = 0
        expected = 'Missing or out-of-range motor limits/timing'
    elif case == 'controlled_bench_zero_rate':
        profile['GP_MAX_STEP_HZ'] = 0
        expected = 'Missing or out-of-range motor limits/timing'
    else:
        raise SystemExit('unknown case')
elif case in ('signal_profile', 'signal_watchdog', 'signal_missing_mode'):
    profile = {'GP_ENABLE_CAN': 1, 'GP_ENABLE_ACTUATION': 1,
               'GP_UNPOWERED_SIGNAL_TEST': 1}
    if case == 'signal_watchdog':
        profile['GP_ENABLE_WATCHDOG'] = 1
        expected = 'SignalTest requires unpowered waveform mode'
    elif case == 'signal_missing_mode':
        profile['GP_UNPOWERED_SIGNAL_TEST'] = 0
        expected = 'SignalTest requires unpowered waveform mode'
elif case == 'component_timing':
    profile['GP_STEP_PULSE_US'] = 2
    expected = 'documented BTS7960/EM882S'
elif case in ('bench_profile', 'bench_forbids_actuation'):
    profile = {'GP_ENABLE_CAN': 1}
    if case == 'bench_forbids_actuation':
        profile['GP_ENABLE_ACTUATION'] = 1
        expected = 'CAN bench profile cannot enable motors'
elif case == 'unapproved_can':
    profile = {'GP_ENABLE_CAN': 1}
    expected = 'CAN requires approved pin map'
elif case == 'missing_decoder':
    profile['GP_HAS_DECODER'] = 0
    expected = 'Actuation requires decoder'
elif case == 'unknown_polarity':
    profile.pop('GP_REVERSE_INACTIVE_HIGH')
    expected = 'Actuation requires explicit electrical polarity'
elif case == 'truncated_timing':
    profile['GP_STEP_PULSE_US'] = 65541
    expected = 'no truncated settings allowed'
elif case == 'unapproved_watchdog':
    profile = {'GP_ENABLE_WATCHDOG': 1}
    expected = 'IWDG requires approved reset-safe outputs'
else:
    raise SystemExit('unknown case')

source = '#include "gp_board_config.h"\n'
if case == 'default':
    source += '_Static_assert(GP_PIN_MAP_APPROVED && !GP_ENABLE_CAN && !GP_ENABLE_ACTUATION && !GP_ENABLE_WATCHDOG && GP_HAS_DECODER, "final wiring stays inhibited");\n'
elif case == 'controlled_bench_template':
    source += '''
_Static_assert(GP_API_REVISION == 10, "canonical API");
_Static_assert(GP_ENABLE_ACTUATION && !GP_UNPOWERED_SIGNAL_TEST, "real actuation mode");
_Static_assert(!GP_BENCH_COMMISSIONING_APPROVED && !GP_STEERING_EXTERNAL_READY_BEFORE_ARM, "no fake approvals");
_Static_assert(GP_THROTTLE_CAP_PM == 1000 && GP_STEERING_CAP_PM == 1000, "full command range");
_Static_assert(GP_MAX_STEP_HZ == 166666 && GP_STEP_PULSE_US == 3 && GP_STEP_LOW_US == 3, "integer-us interface ceiling");
'''
    source = '#include "gp_types.h"\n' + source
command = [args.cc, '-std=c11', '-x', 'c', '-fsyntax-only', '-I' + str(root / 'App/Inc'),
           '-I' + str(root / 'Core/Inc')]
if case in ('bench_profile', 'bench_forbids_actuation'):
    command += ['-include', str(root / 'App/Profiles/can_bench.h')]
if case in ('signal_profile', 'signal_watchdog', 'signal_missing_mode'):
    command += ['-include', str(root / 'App/Profiles/signal_test.h')]
if case == 'controlled_bench_template':
    command += ['-include', str(root / 'App/Profiles/bench_actuation.h')]
command += ['-D' + name + '=' + str(value) for name, value in profile.items()]
result = subprocess.run(command + ['-'], input=source, text=True, capture_output=True)
if expected:
    if result.returncode == 0 or expected not in result.stderr:
        raise SystemExit('Expected rejection missing: ' + expected + '\n' + result.stderr)
elif result.returncode != 0:
    raise SystemExit(result.stderr)
print('PASS', case)
