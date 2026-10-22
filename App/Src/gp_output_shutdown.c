#include "gp_stm32_bsp.h"
#include "gp_board_config.h"

void GpBsp_EmergencyOff(void)
{
    /* Register-only best effort, also usable before BSP initialization and from
     * a fatal exception. No HAL/RTOS calls, readiness assumptions or power cutoff.
     * A damaged CPU/stack or failed GPIO still requires external interruption. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
#if GP_ENABLE_ACTUATION
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = (GP_PWM_ACTIVE_HIGH ? (GPIO_PIN_3 << 16u) : GPIO_PIN_3) |
                  (GP_REVERSE_INACTIVE_HIGH ? GPIO_PIN_4 : (GPIO_PIN_4 << 16u));
    GPIOA->BSRR = GP_STEP_ACTIVE_HIGH ? (GPIO_PIN_6 << 16u) : GPIO_PIN_6;
    /* Select the already-inactive GPIO latch BEFORE disconnecting AF output.
     * GPIO mode 0x2 = 2 MHz push-pull. DIR and unrelated pins are held. */
    GPIOB->CRL = (GPIOB->CRL & ~((0xfu << 12u) | (0xfu << 16u))) |
                 (0x2u << 12u) | (0x2u << 16u);
    GPIOA->CRL = (GPIOA->CRL & ~(0xfu << 24u)) | (0x2u << 24u);
#endif
    TIM2->CCER &= ~TIM_CCER_CC2E;
    TIM3->CCER &= ~TIM_CCER_CC1E;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CCR2 = TIM3->CCR1 = 0;
}

