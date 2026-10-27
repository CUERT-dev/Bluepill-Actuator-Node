"""Type-check enabled STM32 paths with synthetic settings; do not link an image."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--cc', required=True)
p.add_argument('--root', required=True)
args = p.parse_args()
root = Path(args.root).resolve()
includes = ['App/Inc', 'Core/Inc', 'lib/control/include', 'lib/hardware/include',
            'lib/motor_driver/include', 'Drivers/STM32F1xx_HAL_Driver/Inc',
            'Drivers/CMSIS/Device/ST/STM32F1xx/Include', 'Drivers/CMSIS/Include',
            'Middlewares/Third_Party/FreeRTOS/Source/include',
            'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3']
command = [args.cc, '-mcpu=cortex-m3', '-mthumb', '-std=c11', '-Wall', '-Wextra',
           '-Werror', '-Wpedantic', '-fsyntax-only', '-DUSE_HAL_DRIVER', '-DSTM32F103xB',
           '-DUSE_FULL_ASSERT',
           '-DHAL_CAN_MODULE_ENABLED', '-DHAL_TIM_MODULE_ENABLED', '-DHAL_IWDG_MODULE_ENABLED',
           '-include', str(root / 'tests/fixtures/compile_only_board.h')]
command += ['-I' + str(root / directory) for directory in includes]
for fixture in ('compile_only_board.h', 'compile_only_bench.h'):
    checked = [str(root / 'tests/fixtures' / fixture) if arg ==
               str(root / 'tests/fixtures/compile_only_board.h') else arg for arg in command]
    if fixture == 'compile_only_bench.h':
        checked.append('-DGP_DIRECT_COMMANDS=1')
    for source in ('App/Src/gp_stm32_bsp.c', 'App/Src/gp_output_shutdown.c', 'App/Src/gp_app.c', 'Core/Src/stm32f1xx_it.c',
                   'Core/Src/main.c', 'App/Src/gp_diag.c', 'App/Src/gp_diag_format.c', 'App/Src/gp_service.c'):
        subprocess.run(checked + [str(root / source)], check=True)
print('PASS signal/controlled-bench, CAN, watchdog, tasks, ISR and assert syntax; no synthetic image linked')
