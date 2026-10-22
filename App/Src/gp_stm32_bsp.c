#include "gp_stm32_bsp.h"
#include "gp_board_config.h"
#include "gp_app.h"
#include <string.h>

volatile GpBootStatus gp_boot_status;
static CAN_HandleTypeDef hcan;
static TIM_HandleTypeDef htim2, htim3;
static IWDG_HandleTypeDef hiwdg;
static GpMotorHw motor_hw;
static bool timer_ready;
static uint32_t cycle_previous, cycle_remainder, microseconds;

static uint32_t timer_clock(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    return (RCC->CFGR & RCC_CFGR_PPRE1) ? clock * 2u : clock;
}

#if GP_ENABLE_ACTUATION
static void output_gpio(GPIO_TypeDef *gpio, uint16_t pin, bool high, bool alternate)
{
    GPIO_InitTypeDef init = {0};
    HAL_GPIO_WritePin(gpio, pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    init.Pin = pin;
    init.Mode = alternate ? GPIO_MODE_AF_PP : GPIO_MODE_OUTPUT_PP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(gpio, &init);
}
#endif

static void raw_off(void *user)
{
    (void)user;
    GpBsp_EmergencyOff();
}

void GpBsp_Init(void)
{
    gp_boot_status.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* SWD PA13/14 retained */
#if GP_ENABLE_ACTUATION
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1(); /* final PB3/L-PWM; forward still unknown */
    __HAL_AFIO_REMAP_TIM3_DISABLE();   /* final PA6/STEP; PB4 stays GPIO */
#endif
    raw_off(NULL);
    /* No AF PWM/STEP route is selected in the inhibited build. */
#if !GP_ENABLE_ACTUATION
    GPIO_InitTypeDef analog = {0};
    analog.Mode = GPIO_MODE_ANALOG;
    analog.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &analog);
    analog.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &analog);
    gp_boot_status.blockers |= GP_BLOCK_ACTUATION;
#endif
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycle_previous = DWT->CYCCNT;
    hcan.Instance = CAN1;
    htim2.Instance = TIM2;
    htim3.Instance = TIM3;
}

uint32_t GpBsp_Micros(void)
{
    /* Motor-task-owned DWT extension. Must be serviced within one 32-bit cycle
     * wrap (~59 s at 72 MHz). Dividing CYCCNT directly would wrap incorrectly. */
    uint32_t cycles = DWT->CYCCNT;
    uint32_t delta = cycles - cycle_previous;
    cycle_previous = cycles;
    if (!SystemCoreClock) return microseconds;
    uint64_t total = (uint64_t)delta * 1000000u + cycle_remainder;
    microseconds += (uint32_t)(total / SystemCoreClock);
    cycle_remainder = (uint32_t)(total % SystemCoreClock);
    return microseconds;
}
static uint32_t now_us(void *user) { (void)user; return GpBsp_Micros(); }

static bool permitted(void *user)
{
    (void)user;
#if GP_ENABLE_ACTUATION
    /* Software waveform permission only; this board has no safety readbacks. */
    return timer_ready && (GP_UNPOWERED_SIGNAL_TEST || GP_CONTROLLED_BENCH_ACTUATION);
#else
    return false;
#endif
}

static bool direction(void *user, int8_t sign)
{
    (void)user;
    if (!permitted(NULL) || (sign != 1 && sign != -1)) return false;
#if GP_ENABLE_ACTUATION
    bool high = (sign > 0) == (GP_DIR_POSITIVE_HIGH != 0);
    output_gpio(GPIOA, GPIO_PIN_7, high, false);
    return true;
#else
    return false;
#endif
}

#if GP_ENABLE_ACTUATION
static void commit_timer(TIM_TypeDef *tim, const GpTimerPlan *plan, bool step)
{
    uint32_t enable = step ? TIM_CCER_CC1E : TIM_CCER_CC2E;
    bool running = (tim->CR1 & TIM_CR1_CEN) != 0;
    /* PSC, ARR and CCR are preloaded. Existing pulse widths are preserved until
     * the next natural update; do not force UG on a running pulse train. */
    /* Prevent an overflow between these stores from latching a mixed plan.
     * Counter still runs; preload transfer resumes at the next update. */
    uint32_t udis = tim->CR1 & TIM_CR1_UDIS;
    tim->CR1 |= TIM_CR1_UDIS;
    tim->PSC = plan->prescaler;
    tim->ARR = plan->autoreload;
    if (step) tim->CCR1 = plan->compare; else tim->CCR2 = plan->compare;
    if (!udis) tim->CR1 &= ~TIM_CR1_UDIS;
    if (!running) {
        tim->EGR = TIM_EGR_UG;
        /* Start STEP from the final low tick. GPIO is connected while the
         * reference is inactive, then CEN starts a complete first pulse. */
        tim->CNT = plan->autoreload;
        tim->SR = 0;
        tim->CCER |= enable;
    }
}
#endif

static bool apply_timers(void *user, const GpTimerPlan *pwm, const GpTimerPlan *step)
{
    (void)user;
    if (!permitted(NULL) || !pwm) return false;
#if GP_ENABLE_ACTUATION
    if (pwm->compare) {
        bool running = (TIM2->CR1 & TIM_CR1_CEN) != 0;
        commit_timer(TIM2, pwm, false);
        if (!running) output_gpio(GPIOB, GPIO_PIN_3, GP_PWM_ACTIVE_HIGH == 0, true);
        TIM2->CR1 |= TIM_CR1_CEN;
    } else {
        TIM2->CCER &= ~TIM_CCER_CC2E;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        output_gpio(GPIOB, GPIO_PIN_3, GP_PWM_ACTIVE_HIGH == 0, false);
    }
    if (step) {
        bool running = (TIM3->CR1 & TIM_CR1_CEN) != 0;
        commit_timer(TIM3, step, true);
        if (!running) output_gpio(GPIOA, GPIO_PIN_6, GP_STEP_ACTIVE_HIGH == 0, true);
        TIM3->CR1 |= TIM_CR1_CEN;
    } else {
        TIM3->CCER &= ~TIM_CCER_CC1E;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        output_gpio(GPIOA, GPIO_PIN_6, GP_STEP_ACTIVE_HIGH == 0, false);
    }
    return true;
#else
    (void)step;
    return false;
#endif
}

static bool init_timers(void)
{
    TIM_OC_InitTypeDef oc = {0};
    htim2.Init.Prescaler = htim3.Init.Prescaler = 0;
    htim2.Init.CounterMode = htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = htim3.Init.Period = UINT16_MAX;
    htim2.Init.ClockDivision = htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK || HAL_TIM_PWM_Init(&htim3) != HAL_OK) return false;
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = GP_PWM_ACTIVE_HIGH == 1 ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse = 0;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2) != HAL_OK) return false;
    oc.OCPolarity = GP_STEP_ACTIVE_HIGH == 1 ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK) return false;
    raw_off(NULL); /* never call HAL_TIM_PWM_Start at initialization */
    return true;
}

GpResult GpBsp_MotorConfigure(CommandMapConfig *mapping, MotorDriverConfig *driver)
{
    if (!mapping || !driver) return GP_BAD_ARG;
    memset(mapping, 0, sizeof(*mapping));
    memset(driver, 0, sizeof(*driver));
    if (!GP_ENABLE_ACTUATION || !GP_OUTPUT_SETTINGS_CONFIGURED || !GP_THROTTLE_CAP_PM ||
        !GP_STEERING_CAP_PM || GP_STEERING_CAP_PM > 1000 || !GP_THROTTLE_SLEW_PM_S ||
        !GP_STEERING_SLEW_PM_S || !GP_REVERSAL_DEADTIME_MS) {
        gp_boot_status.blockers |= GP_BLOCK_MOTOR_SETTINGS;
        return GP_INVALID;
    }
    GpMotorHwConfig config = {timer_clock(), GP_THROTTLE_PWM_HZ, GP_MAX_STEP_HZ,
        GP_THROTTLE_CAP_PM, GP_STEP_PULSE_US, GP_STEP_LOW_US,
        GP_DIR_SETUP_US, GP_ENABLE_SETUP_US, true};
    /* ENA is not wired: no physical enable transaction can be controlled or
     * verified here. DIR-to-STEP setup remains in the adapter. Controlled bench
     * commissioning must establish stable driver power/enable BEFORE arming;
     * loss of that external condition requires cutoff and a verified startup before rearming. */
    GpMotorHwPort port = {NULL, now_us, raw_off, direction, apply_timers, permitted, NULL};
    if (GpMotorHw_Init(&motor_hw, &config, &port) != GP_OK || !init_timers()) return GP_INVALID;
    timer_ready = true;
    *mapping = (CommandMapConfig){GP_THROTTLE_CAP_PM, GP_STEERING_CAP_PM,
        GP_THROTTLE_SLEW_PM_S, GP_STEERING_SLEW_PM_S, GP_MAPPER_MAX_DT_MS};
    *driver = (MotorDriverConfig){GpBsp_MotorWrite, GpBsp_MotorOff, NULL,
        GP_THROTTLE_CAP_PM, GP_MAX_STEP_HZ, GP_STEP_PULSE_US,
        GP_DIR_SETUP_US, GP_ENABLE_SETUP_US, GP_REVERSAL_DEADTIME_MS};
    return GP_OK;
}

GpResult GpBsp_MotorWrite(void *user, const MotorOutput *output)
{
    (void)user;
    return GpMotorHw_Write(&motor_hw, output);
}
void GpBsp_MotorOff(void *user)
{
    (void)user;
    raw_off(NULL);
    GpMotorHw_Off(&motor_hw);
}

GpResult GpBsp_CanStart(void)
{
    gp_boot_status.blockers |= GP_BLOCK_CAN;
    if (!GP_ENABLE_CAN || !GP_HAS_DECODER ||
        !GpCanTiming_Valid(HAL_RCC_GetPCLK1Freq(), GP_CAN_BITRATE, GP_CAN_PRESCALER,
                           GP_CAN_BS1_TQ, GP_CAN_BS2_TQ, GP_CAN_SJW_TQ)) return GP_INVALID;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_AFIO_REMAP_CAN1_2();
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    hcan.Init.Prescaler = GP_CAN_PRESCALER;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = (GP_CAN_SJW_TQ - 1u) << 24;
    hcan.Init.TimeSeg1 = (GP_CAN_BS1_TQ - 1u) << 16;
    hcan.Init.TimeSeg2 = (GP_CAN_BS2_TQ - 1u) << 20;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE; /* no unapproved automatic recovery */
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = ENABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) return GP_IO_ERROR;
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = GP_CAN_COMMAND_ID << 5;
    filter.FilterMaskIdHigh = 0x7ffu << 5;
    filter.FilterMaskIdLow = (1u << 2) | (1u << 1); /* standard data only */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK || HAL_CAN_Start(&hcan) != HAL_OK) return GP_IO_ERROR;
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, GP_CAN_IRQ_PRIORITY, 0);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, GP_CAN_IRQ_PRIORITY, 0);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, GP_CAN_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    gp_boot_status.can_ready = true;
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_BUSOFF |
        CAN_IT_ERROR | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
        gp_boot_status.can_ready = false;
        return GP_IO_ERROR;
    }
    gp_boot_status.blockers &= ~GP_BLOCK_CAN;
    return GP_OK;
}

void GpBsp_CanInterrupt(void)
{
    if (!gp_boot_status.can_ready) return;
    if ((CAN1->RF0R & CAN_RF0R_FOVR0) || (CAN1->RF1R & CAN_RF1R_FOVR1)) {
        GpApp_LatchCanFault(GP_FAULT_CAN_OVERFLOW);
    }
    if (CAN1->ESR & CAN_ESR_BOFF) GpApp_LatchCanFault(GP_FAULT_BUS_OFF);
    HAL_CAN_IRQHandler(&hcan);
}
GpResult GpBsp_CanRestart(void)
{
    if (!GP_ENABLE_CAN) return GP_INVALID;
    gp_boot_status.blockers |= GP_BLOCK_CAN;
    /* Explicit recovery only. HAL transition waits are outside the core lock
     * and only occur after motor shutdown; there is no automatic bus-off retry. */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    gp_boot_status.can_ready = false;
    (void)HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_BUSOFF |
        CAN_IT_ERROR | CAN_IT_LAST_ERROR_CODE);
    /* Recover even from a previous initialization/restart failure. Peripheral
     * reset below is authoritative; Stop can legitimately fail if not started. */
    (void)HAL_CAN_Stop(&hcan);
    if (HAL_CAN_DeInit(&hcan) != HAL_OK) return GP_IO_ERROR;
    __HAL_RCC_CAN1_FORCE_RESET();
    __HAL_RCC_CAN1_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_ClearPendingIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_ClearPendingIRQ(CAN1_SCE_IRQn);
    return GpBsp_CanStart();
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    for (uint32_t i = 0; i < GP_CAN_ISR_DRAIN_BUDGET; ++i) {
        if (!HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0)) break;
        CAN_RxHeaderTypeDef header = {0};
        GpCanFrame frame = {0};
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            GpApp_LatchCanFault(GP_FAULT_CAN_OVERFLOW);
            break;
        }
        frame.id = (uint16_t)header.StdId;
        frame.dlc = (uint8_t)header.DLC;
        if (GpApp_CaptureFrame(&frame, header.IDE != CAN_ID_STD, header.RTR != CAN_RTR_DATA) == GP_NO_SPACE) {
            /* Avoid an RX interrupt storm starving the urgent motor shutdown.
             * Recovery needs a deliberate controller restart after off. */
            (void)HAL_CAN_DeactivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING |
                CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN);
            break;
        }
    }
    /* A still-pending FIFO retriggers the IRQ; no unbounded ISR drain loop. */
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    uint32_t errors = HAL_CAN_GetError(can);
    if (errors & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) GpApp_LatchCanFault(GP_FAULT_CAN_OVERFLOW);
    if (errors & HAL_CAN_ERROR_BOF) GpApp_LatchCanFault(GP_FAULT_BUS_OFF);
}

bool GpBsp_DiscardRx(void *user)
{
    (void)user;
    if (!gp_boot_status.can_ready) return true;
    for (uint32_t fifo = 0; fifo < 2; ++fifo) {
        for (uint32_t i = 0; i < 3; ++i) {
            if (!HAL_CAN_GetRxFifoFillLevel(&hcan, fifo)) break;
            CAN_RxHeaderTypeDef header;
            uint8_t data[8];
            if (HAL_CAN_GetRxMessage(&hcan, fifo, &header, data) != HAL_OK) return false;
        }
    }
    /* Do not erase a hardware overrun that the ISR has not yet latched. */
    return HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0 &&
           HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1) == 0 &&
           !(CAN1->RF0R & CAN_RF0R_FOVR0) && !(CAN1->RF1R & CAN_RF1R_FOVR1);
}
bool GpBsp_ControlInputs(void *user, GpControlInputs *inputs)
{
    (void)user;
    if (!inputs) return false;
    *inputs = (GpControlInputs){.estop_mode = GP_ESTOP_NOT_CONNECTED};
    inputs->can_bus_ok = gp_boot_status.can_ready && !(CAN1->ESR & CAN_ESR_BOFF);
    inputs->hardware_rx_pending = gp_boot_status.can_ready &&
        ((CAN1->RF0R & (CAN_RF0R_FMP0 | CAN_RF0R_FOVR0)) ||
         (CAN1->RF1R & (CAN_RF1R_FMP1 | CAN_RF1R_FOVR1)));
#if GP_ENABLE_ACTUATION
    inputs->outputs_approved = permitted(NULL) && (!GP_ENABLE_WATCHDOG || gp_boot_status.watchdog_started);
#endif
    return true;
}

bool GpBsp_WatchdogStart(void)
{
    gp_boot_status.blockers |= GP_BLOCK_WATCHDOG;
    if (!GP_ENABLE_WATCHDOG) {
        return false;
    }
    if (!GP_WATCHDOG_TASK_DEADLINE_MS || GP_WATCHDOG_TASK_DEADLINE_MS >= 0x80000000u ||
        GP_IWDG_RELOAD < 1 || GP_IWDG_RELOAD > 4095 ||
        (GP_IWDG_PRESCALER != 4 && GP_IWDG_PRESCALER != 8 && GP_IWDG_PRESCALER != 16 &&
         GP_IWDG_PRESCALER != 32 && GP_IWDG_PRESCALER != 64 && GP_IWDG_PRESCALER != 128 &&
         GP_IWDG_PRESCALER != 256) || !GP_IWDG_LSI_MAX_HZ ||
        (uint64_t)(GP_IWDG_RELOAD + 1u) * GP_IWDG_PRESCALER * 1000u <=
        (uint64_t)(GP_WATCHDOG_TASK_DEADLINE_MS + GP_MOTOR_PERIOD_MS) * GP_IWDG_LSI_MAX_HZ) return false;
    uint32_t encoding = 0, prescaler = GP_IWDG_PRESCALER;
    while (prescaler > 4) { prescaler /= 2; ++encoding; } /* at most six steps */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = encoding;
    hiwdg.Init.Reload = GP_IWDG_RELOAD;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) GpApp_Fatal(6);
    gp_boot_status.watchdog_started = true;
    gp_boot_status.blockers &= ~GP_BLOCK_WATCHDOG;
    return true;
}
void GpBsp_WatchdogFeed(void)
{
    if (gp_boot_status.watchdog_started) (void)HAL_IWDG_Refresh(&hiwdg);
}
