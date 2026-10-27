#include "gp_diag.h"
#include "gp_service.h"
#include "gp_app.h"
#include "gp_board_config.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* USART1 115200 8N1. Single producer/consumer rings; no RTOS API in IRQ.
 * Diagnostic task priority 1, fixed 16 RX byte budget per 1 ms wake.
 * TX queue drops whole responses when busy. It never delays motor/CAN tasks. */
#define RX_SIZE 64u
#define TX_SIZE 1536u
static volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
static volatile uint32_t rx_errors, tx_dropped;
static uint8_t rx[RX_SIZE], tx[TX_SIZE];
static StaticTask_t diag_tcb;
static StackType_t diag_stack[768];
static GpServiceParser parser;
static char response[640];

static void send_text(const char *s)
{
    size_t n = strlen(s);
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint16_t free_bytes = (uint16_t)((tx_tail + TX_SIZE - tx_head - 1u) % TX_SIZE);
    if (n > free_bytes) {
        ++tx_dropped;
        __set_PRIMASK(saved);
        return;
    }
    /* Reserve by writing before publishing head. ISR can consume old data
     * while the task copies. Only this task produces responses. */
    uint16_t head = tx_head;
    __set_PRIMASK(saved);
    for (size_t i = 0; i < n; ++i) {
        tx[head] = (uint8_t)s[i];
        head = (uint16_t)((head + 1u) % TX_SIZE);
    }
    saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tx_head = head;
    USART1->CR1 |= USART_CR1_TXEIE;
    __set_PRIMASK(saved);
}
static void status(void)
{
    GpDiagStatus s;
    GpApp_GetStatus(&s);
    int n = snprintf(response, sizeof(response),
        "ms=%lu can=%u motor=%u wd=%u state=%lu fault=%08lx history=%08lx "
        "block=%08lx reset=%08lx panic=%lu rx=%lu drop=%lu decode=%lu reject=%lu "
        "stale=%lu publish=%lu barred=%lu decoded_ms=%lu seq=%u flags=%u "
        "decoded=%u,%d requested=%u,%d applied=%d,%d recover=%lu,%ld "
        "btr=%08lx esr=%08lx uart_error=%lu tx_drop=%lu\r\n",
        (unsigned long)s.uptime_ms, s.can_ready, s.motor_ready, s.watchdog,
        (unsigned long)s.state, (unsigned long)s.faults, (unsigned long)s.history,
        (unsigned long)s.blockers, (unsigned long)s.reset_flags, (unsigned long)s.panic,
        (unsigned long)s.captured, (unsigned long)s.dropped, (unsigned long)s.decoded,
        (unsigned long)s.rejected, (unsigned long)s.stale, (unsigned long)s.published,
        (unsigned long)s.barred, (unsigned long)s.decoded_ms,
        s.decoded_command.sequence, s.decoded_command.flags,
        s.decoded_command.throttle_pm, s.decoded_command.steering_pm,
        s.requested.throttle_pm, s.requested.steering_pm,
        s.applied.throttle_duty_pm, s.applied.steering_rate_pm,
        (unsigned long)s.recovery_requests, (long)s.recovery_result,
        (unsigned long)s.can_btr, (unsigned long)s.can_esr,
        (unsigned long)rx_errors, (unsigned long)tx_dropped);
    if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
    n = GpDiag_FormatMotion(response, sizeof(response), &s);
    if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
    n = GpDiag_FormatRx(response, sizeof(response), &s);
    if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
}
static void entry(void *unused)
{
    (void)unused;
    uint32_t errors_seen = 0;
#if GP_UNPOWERED_SIGNAL_TEST
    send_text("UNPOWERED SIGNAL TEST: disconnect motor power; no MCU E-stop/interlock/readiness inputs\r\n");
#elif GP_CONTROLLED_BENCH_ACTUATION
    send_text("COMMISSIONING CANDIDATE: actuation enabled; provisional settings; no safety/readiness inputs\r\n");
#endif
    send_text("CUERT service 115200 8N1: status / recover + newline; no motor commands\r\n");
#if GP_DIRECT_COMMANDS
    send_text("DIRECT COMMANDS: no ARM/DEADMAN or neutral arming; first valid demand can move outputs; STOP/timeout/fault shutdown retained\r\n");
#endif
    send_text("BLUE FW actuation-diag-v5: CAN TX=PB9 RX=PB8; command ID=0x105 DLC=8; reports every 1s\r\n");
#if GP_DIRECT_COMMANDS
    send_text("DIRECT RESUME: fresh valid CAN resumes after timeout/queue-age/backlog/timing stop\r\n");
#endif
    int startup = snprintf(response, sizeof(response),
        "PROFILE can=%u actuation=%u signal=%u bench=%u wd=%u; "
        "t2/t3=PSC,ARR,CCR,CR1,CCER; pa/pb=CRL,IDR,ODR\r\n",
        GP_ENABLE_CAN, GP_ENABLE_ACTUATION, GP_UNPOWERED_SIGNAL_TEST,
        GP_CONTROLLED_BENCH_ACTUATION, GP_ENABLE_WATCHDOG);
    if (startup > 0 && (size_t)startup < sizeof(response)) send_text(response);
#if GP_ENABLE_ACTUATION
    startup = snprintf(response, sizeof(response),
        "STEP full_scale_hz=%lu slew_pm_s=%lu\r\n",
        (unsigned long)GP_MAX_STEP_HZ, (unsigned long)GP_STEERING_SLEW_PM_S);
    if (startup > 0 && (size_t)startup < sizeof(response)) send_text(response);
#endif
#if GP_ENABLE_CAN
    send_text("BLUE CAN configured ON at 500000 bit/s; inspect ready=1 and rising rx/decode\r\n");
#else
    send_text("BLUE CAN DISABLED in this image: flash CanBench to test the ESP link\r\n");
#endif
    uint32_t last_report = HAL_GetTick();
    for (;;) {
        for (unsigned i = 0; i < 16; ++i) {
            uint32_t saved = __get_PRIMASK();
            __disable_irq();
            uint32_t errors = rx_errors;
            if (errors != errors_seen) {
                /* Discard all buffered bytes on loss before parsing a command. */
                rx_tail = rx_head;
                parser.discard = true;
                errors_seen = errors;
            }
            bool have = rx_tail != rx_head;
            uint8_t byte = 0;
            if (have) {
                byte = rx[rx_tail];
                rx_tail = (uint16_t)((rx_tail + 1u) % RX_SIZE);
            }
            __set_PRIMASK(saved);
            if (!have) break;
            GpServiceCommand command = GpService_Byte(&parser, byte, false);
            if (command == GP_SERVICE_STATUS) status();
            else if (command == GP_SERVICE_RECOVER) {
                GpApp_RequestRecovery();
#if GP_DIRECT_COMMANDS
                send_text("recovery queued; inspect status; after recovery next fresh valid command can drive outputs\r\n");
#else
                send_text("recovery queued; inspect status; fresh neutral/ARM edge still required\r\n");
#endif
            } else if (command == GP_SERVICE_BAD_COMMAND) send_text("ERR command\r\n");
        }
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_report) >= 1000u) {
            GpDiagStatus s;
            GpApp_GetStatus(&s);
            int n = snprintf(response, sizeof(response),
                "BLUE CAN ready=%u configured=%u bitrate=%lu rx=%lu decode=%lu reject=%lu drop=%lu "
                "seq=%u thr=%u str=%d flags=0x%02X age_ms=%lu ESR=0x%08lX BTR=0x%08lX fault=0x%08lX motor=%u\r\n",
                s.can_ready, GP_ENABLE_CAN, (unsigned long)GP_CAN_BITRATE,
                (unsigned long)s.captured, (unsigned long)s.decoded,
                (unsigned long)s.rejected, (unsigned long)s.dropped,
                (unsigned)s.decoded_command.sequence, (unsigned)s.decoded_command.throttle_pm,
                (int)s.decoded_command.steering_pm, (unsigned)s.decoded_command.flags,
                (unsigned long)(now - s.decoded_ms), (unsigned long)s.can_esr,
                (unsigned long)s.can_btr, (unsigned long)s.faults, s.motor_ready);
            if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
            n = GpDiag_FormatMotion(response, sizeof(response), &s);
            if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
            n = GpDiag_FormatRx(response, sizeof(response), &s);
            if (n > 0 && (size_t)n < sizeof(response)) send_text(response);
            last_report = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void GpDiag_Start(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_DISABLE();
    GPIO_InitTypeDef gpio = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_9; gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    USART1->CR1 = 0; USART1->CR2 = 0; USART1->CR3 = USART_CR3_EIE;
    /* Oversampling by 16: BRR integer encoding is round(PCLK/baud). */
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + 57600u) / 115200u;
    (void)USART1->SR; (void)USART1->DR;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (!xTaskCreateStatic(entry, "Service", 768, NULL, 1, diag_stack, &diag_tcb))
        GpApp_Fatal(7);
}
void GpDiag_Interrupt(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART1->DR; /* SR then DR clears error flags */
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_SIZE);
        if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) || next == rx_tail)
            ++rx_errors;
        else {
            rx[rx_head] = byte;
            __DMB();
            rx_head = next;
        }
    }
    if ((sr & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
        if (tx_tail == tx_head) USART1->CR1 &= ~USART_CR1_TXEIE;
        else {
            USART1->DR = tx[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) % TX_SIZE);
        }
    }
}
