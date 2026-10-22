"""Check actual linked Cortex-M3 vectors and inhibited/signal-profile symbols."""
import argparse
from pathlib import Path
import re
import struct
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--elf', required=True)
p.add_argument('--nm', required=True)
p.add_argument('--objdump', required=True)
p.add_argument('--inhibited', action='store_true')
p.add_argument('--motor-inhibited', action='store_true')
p.add_argument('--can-enabled', action='store_true')
p.add_argument('--signal-test', action='store_true')
p.add_argument('--bench-actuation', action='store_true')
args = p.parse_args()
elf = Path(args.elf)
symbols = {}
bindings = {}
counts = {}
for line in subprocess.check_output([args.nm, '-n', str(elf)], text=True).splitlines():
    fields = line.split()
    if len(fields) == 3:
        symbols[fields[2]] = int(fields[0], 16)
        bindings[fields[2]] = fields[1]
        counts[fields[2]] = counts.get(fields[2], 0) + 1
for name in ('vTaskStartScheduler', 'xTaskCreateStatic', 'GpMotor_Run', 'GpCanRx_Run',
             'MotorDriver_Apply', 'CommandMap_Update', 'vPortSVCHandler',
             'xPortPendSVHandler', 'xPortSysTickHandler', 'GP_DecodeCan', 'GP_EncodeCan',
             'GpDiag_Interrupt', 'GpApp_GetStatus', 'GpMotor_InitInhibited',
             'GpBsp_EmergencyOff'):
    if name not in symbols:
        raise SystemExit('Missing real application/kernel symbol: ' + name)
for name in ('HAL_CAN_RxFifo0MsgPendingCallback', 'HAL_CAN_ErrorCallback'):
    if bindings.get(name) != 'T':
        raise SystemExit('CAN application callback was not linked strongly: ' + name)
for name in ('GP_DecodeCan', 'GP_EncodeCan'):
    if bindings.get(name) != 'T' or counts.get(name) != 1:
        raise SystemExit('Expected exactly one strong production codec symbol: ' + name)
if args.inhibited:
    for name in ('HAL_TIM_PWM_Start', 'HAL_IWDG_Init', 'GpMotorHw_Write'):
        if name in symbols:
            raise SystemExit('Unexpected active/dependency symbol in default profile: ' + name)
if args.motor_inhibited and 'GpMotorHw_Write' in symbols:
    raise SystemExit('Active motor adapter linked into inhibited profile')
if any(name.startswith('GpApproved_') for name in symbols):
    raise SystemExit('Current-board image contains obsolete board-provider symbols')
if args.signal_test or args.bench_actuation:
    for name in ('GpMotorHw_Write', 'GpMotorHw_Off', 'GpBsp_MotorWrite',
                 'HAL_TIM_PWM_Init', 'HAL_TIM_PWM_ConfigChannel'):
        if bindings.get(name) != 'T':
            raise SystemExit('Active output profile lacks real motor adapter/HAL: ' + name)
    if args.signal_test and 'HAL_IWDG_Init' in symbols:
        raise SystemExit('SignalTest unexpectedly starts watchdog')
if args.can_enabled:
    for name in ('HAL_CAN_Init', 'HAL_CAN_Start', 'HAL_CAN_ConfigFilter', 'HAL_CAN_ActivateNotification'):
        if bindings.get(name) != 'T':
            raise SystemExit('CAN bench lacks real HAL operation: ' + name)
fatal = subprocess.check_output([args.objdump, '-d', '--disassemble=GpApp_Fatal', str(elf)], text=True)
if '<GpBsp_EmergencyOff>' not in fatal:
    raise SystemExit('Fatal path does not call the register-only output shutdown')

data = elf.read_bytes()
if data[:6] != b'\x7fELF\x01\x01':
    raise SystemExit('Expected little-endian ELF32')
shoff = struct.unpack_from('<I', data, 32)[0]
shentsize, shnum, shstridx = struct.unpack_from('<HHH', data, 46)
headers = [struct.unpack_from('<10I', data, shoff + i * shentsize) for i in range(shnum)]
strings_hdr = headers[shstridx]
strings = data[strings_hdr[4]:strings_hdr[4] + strings_hdr[5]]
vectors = None
for header in headers:
    name = strings[header[0]:].split(b'\0', 1)[0]
    if name == b'.isr_vector':
        vectors = data[header[4]:header[4] + header[5]]
        break
if vectors is None:
    raise SystemExit('Vector table missing')
for index, name in ((11, 'SVC_Handler'), (14, 'PendSV_Handler'), (15, 'SysTick_Handler'),
                    (36, 'USB_LP_CAN1_RX0_IRQHandler'), (37, 'CAN1_RX1_IRQHandler'),
                    (38, 'CAN1_SCE_IRQHandler'), (53, 'USART1_IRQHandler')):
    address = struct.unpack_from('<I', vectors, index * 4)[0]
    if not address & 1 or address & ~1 != symbols.get(name):
        raise SystemExit('Bad vector: ' + name)
for name, target in (('SVC_Handler', 'vPortSVCHandler'), ('PendSV_Handler', 'xPortPendSVHandler')):
    disassembly = subprocess.check_output([args.objdump, '-d', '--disassemble=' + name, str(elf)], text=True)
    block = disassembly.split('<' + name + '>:', 1)[-1]
    instructions = [line for line in block.splitlines() if re.match(r'\s*[0-9a-f]+:', line)]
    if not instructions or not re.search(r'\bb(?:\.w)?\s+.*<' + target + '>', instructions[0]):
        raise SystemExit('Exception forwarder changed the stack before branch: ' + name)
print('PASS linked kernel, control, exception vectors, and profile checks')
