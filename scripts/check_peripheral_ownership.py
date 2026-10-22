"""CubeMX owns clock/SWD startup; gated BSP owns CAN/TIM and service owns UART.
Reject accidental generated startup calls that would bypass the profile gates.
"""
import argparse
from pathlib import Path
import re

p = argparse.ArgumentParser()
p.add_argument('--root', default='.')
p.add_argument('--src', default='Core/Src')
a = p.parse_args()
root = Path(a.root)
ioc = (root / 'Bluepill-Actuator-Node.ioc').read_text()
for setting in ('RCC.SYSCLKFreq_VALUE=72000000', 'RCC.APB1Freq_Value=36000000',
                'RCC.APB1TimFreq_Value=72000000', 'PA13.Mode=Serial_Wire',
                'PA14.Mode=Serial_Wire', 'ProjectManager.KeepUserCode=true'):
    if setting not in ioc:
        raise SystemExit('Clock/SWD ownership changed: ' + setting)
source = root / a.src
for path in source.glob('*.c'):
    text = path.read_text()
    if re.search(r'\bMX_(?:CAN\d*|TIM[23]|USART1_UART|IWDG|FREERTOS)_Init\s*\(', text):
        raise SystemExit('Conflicting generated peripheral/RTOS startup: ' + str(path))
print('PASS clock/SWD and exclusive gated peripheral/scheduler ownership')
