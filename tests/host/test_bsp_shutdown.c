/* Execute the actual BSP register shutdown against mapped register storage.
 * This checks stores/masks and cancellation, not electrical output behavior. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#undef ERROR /* CMSIS declares ErrorStatus.ERROR. */
#else
#include <sys/mman.h>
#endif
#include "gp_stm32_bsp.h" /* After native headers: CMSIS reserves __I/CRC. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%d: %s\n", __LINE__, #x); exit(1); } } while (0)
int main(void)
{
    const uintptr_t address = 0x40000000u;
    const size_t bytes = 0x30000u;
#ifdef _WIN32
    void *mapped = VirtualAlloc((void *)address, bytes, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
    void *mapped = mmap((void *)address, bytes, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
#endif
    CHECK(mapped == (void *)address);
    memset(mapped, 0, bytes);
    GPIOA->CRL = GPIOB->CRL = 0xffffffffu;
    TIM2->CCER = TIM3->CCER = 0xffffu;
    TIM2->CR1 = TIM3->CR1 = 0xffffu;
    TIM2->CCR2 = TIM3->CCR1 = 123;
    TIM2->CCR1 = TIM3->CCR2 = 456;
    GpBsp_EmergencyOff();
    CHECK((RCC->APB1ENR & (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN)) ==
                          (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN));
    CHECK((RCC->APB2ENR & (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN)) ==
                          (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN));
    CHECK(TIM2->CCER == (0xffffu & ~TIM_CCER_CC2E));
    CHECK(TIM3->CCER == (0xffffu & ~TIM_CCER_CC1E));
    CHECK(!(TIM2->CR1 & TIM_CR1_CEN) && !(TIM3->CR1 & TIM_CR1_CEN));
    CHECK(!TIM2->CCR2 && !TIM3->CCR1);
    CHECK(TIM2->CCR1 == 456 && TIM3->CCR2 == 456);
    CHECK(GPIOB->BSRR == ((GPIO_PIN_3 | GPIO_PIN_4) << 16u));
    CHECK(GPIOA->BSRR == (GPIO_PIN_6 << 16u));
    CHECK(GPIOB->CRL == 0xfff22fffu); /* PB3/4 output, others untouched */
    CHECK(GPIOA->CRL == 0xf2ffffffu); /* PA6 output; PA7 DIR held */
    GpBsp_EmergencyOff(); /* idempotent; elapsed time cannot restart either timer */
    CHECK(!TIM2->CCR2 && !TIM3->CCR1);
    CHECK(!(TIM2->CR1 & TIM_CR1_CEN) && !(TIM3->CR1 & TIM_CR1_CEN));
#ifdef _WIN32
    CHECK(VirtualFree(mapped, 0, MEM_RELEASE));
#else
    CHECK(munmap(mapped, bytes) == 0);
#endif
    return 0;
}
