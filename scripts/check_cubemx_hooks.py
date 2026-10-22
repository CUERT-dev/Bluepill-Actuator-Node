"""Verify the actual application hooks after isolated CubeMX code generation."""
import argparse
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--src', required=True, help='Generated Src or Core/Src directory')
args = p.parse_args()
checks = {
    'main.c': ['GpBsp_Init();', 'GpApp_Create();', 'GpApp_Fatal(7);',
               'GpApp_AssertFailed((const char *)file, (int)line);'],
    'stm32f1xx_it.c': ['b vPortSVCHandler', 'b xPortPendSVHandler',
                      'GpApp_KernelTick();', 'USB_LP_CAN1_RX0_IRQHandler',
                      'CAN1_RX1_IRQHandler', 'CAN1_SCE_IRQHandler', 'USART1_IRQHandler',
                      'GpDiag_Interrupt();', '__attribute__((naked))'],
    'stm32f1xx_hal_msp.c': ['__HAL_AFIO_REMAP_SWJ_NOJTAG();'],
}
for name, tokens in checks.items():
    data = (Path(args.src) / name).read_text()
    for token in tokens:
        if token not in data:
            raise SystemExit('CubeMX lost hook: ' + name + ': ' + token)
    print('PASS CubeMX preservation:', name)
