#include "gp_diag.h"
#include <stdio.h>

int GpDiag_FormatRx(char *buffer, size_t size, const GpDiagStatus *s)
{
    if (!buffer || !s || !size) return -1;
    const uint8_t *d = s->last_rx_data;
    /* Bytes captured before decoding, including rejected packets. Never use
     * this diagnostic snapshot as an output command or freshness source. */
    const char *sample = s->decoded == 0 ? "NO_VALID_SAMPLE" :
        !(s->decoded_command.flags & GP_FLAG_VALID) ? "STOP" :
        s->decoded_command.throttle_pm == 0 && s->decoded_command.steering_pm == 0
            ? "NEUTRAL" : "DEMAND";
    int result = snprintf(buffer, size,
        "CAN RX have=%u id=%03X dlc=%u ide=%u rtr=%u "
        "data=%02X%02X%02X%02X%02X%02X%02X%02X rx_ms=%lu "
        "last_decoded=%s decoded_ms=%lu\r\n",
        (unsigned)(s->captured != 0), (unsigned)s->last_rx_id,
        (unsigned)s->last_rx_dlc, (unsigned)s->last_rx_extended,
        (unsigned)s->last_rx_remote, (unsigned)d[0], (unsigned)d[1],
        (unsigned)d[2], (unsigned)d[3], (unsigned)d[4], (unsigned)d[5],
        (unsigned)d[6], (unsigned)d[7], (unsigned long)s->last_rx_ms,
        sample, (unsigned long)s->decoded_ms);
    buffer[size - 1u] = '\0';
    return result;
}

int GpDiag_FormatMotion(char *buffer, size_t size, const GpDiagStatus *s)
{
    if (!buffer || !s || !size) return -1;
    /* Register-derived active duty and pulse rate, not electrical measurements.
     * Disabled channels can retain a stale compare value after an axis stop. */
    bool pwm_on = (s->tim2.cr1 & 1u) && (s->tim2.ccer & (1u << 4));
    bool step_on = (s->tim3.cr1 & 1u) && (s->tim3.ccer & 1u) && s->tim3.ccr;
    uint32_t pwm_pm = pwm_on ? (uint32_t)((uint64_t)s->tim2.ccr * 1000u /
                                          ((uint64_t)s->tim2.arr + 1u)) : 0;
    if (pwm_pm > 1000u) pwm_pm = 1000u;
    /* STM32F103 TIM2/TIM3 PSC/ARR are 16-bit. Use 64-bit period arithmetic. */
    uint64_t step_ticks = ((s->tim3.psc & 0xffffu) + 1ull) *
                          ((s->tim3.arr & 0xffffu) + 1ull);
    uint32_t step_hz = step_on ? (uint32_t)(s->timer_clock_hz / step_ticks) : 0;
    int result = snprintf(buffer, size,
        "ACT state=%lu armed=%u direct=%u motor=%u pwm_pm=%lu step_hz=%lu dir=%u "
        "fault=%08lx stop=%08lx stops=%lu runs=%lu run_ms=%lu result=%ld "
        "id=%03x dlc=%u have=%u cmd_ms=%lu received=%u,%d requested=%u,%d applied=%d,%d pending=%u "
        "rx_wait=%u rx_waits=%lu t2=%lu,%lu,%lu,%04lx,%04lx t3=%lu,%lu,%lu,%04lx,%04lx "
        "pa=%08lx,%04lx,%04lx pb=%08lx,%04lx,%04lx mapr=%08lx ena=unwired\r\n",
        (unsigned long)s->state, (unsigned)(s->state == GP_ARMED),
        (unsigned)s->direct_commands, (unsigned)s->motor_ready,
        (unsigned long)pwm_pm, (unsigned long)step_hz, (unsigned)((s->gpioa_odr >> 7) & 1u),
        (unsigned long)s->faults,
        (unsigned long)s->shutdown_reason, (unsigned long)s->shutdown_count,
        (unsigned long)s->motor_runs, (unsigned long)s->motor_ms, (long)s->motor_result,
        (unsigned)s->last_rx_id, (unsigned)s->last_rx_dlc, (unsigned)s->have_requested,
        (unsigned long)s->requested_ms, (unsigned)s->decoded_command.throttle_pm,
        (int)s->decoded_command.steering_pm, (unsigned)s->requested.throttle_pm,
        (int)s->requested.steering_pm, (int)s->applied.throttle_duty_pm,
        (int)s->applied.steering_rate_pm, (unsigned)s->pending,
        (unsigned)s->rx_waiting, (unsigned long)s->rx_wait_count,
        (unsigned long)s->tim2.psc, (unsigned long)s->tim2.arr, (unsigned long)s->tim2.ccr,
        (unsigned long)s->tim2.cr1, (unsigned long)s->tim2.ccer,
        (unsigned long)s->tim3.psc, (unsigned long)s->tim3.arr, (unsigned long)s->tim3.ccr,
        (unsigned long)s->tim3.cr1, (unsigned long)s->tim3.ccer,
        (unsigned long)s->gpioa_crl, (unsigned long)s->gpioa_idr, (unsigned long)s->gpioa_odr,
        (unsigned long)s->gpiob_crl, (unsigned long)s->gpiob_idr, (unsigned long)s->gpiob_odr,
        (unsigned long)s->mapr);
    buffer[size - 1u] = '\0'; /* Also terminate with legacy host C libraries. */
    return result;
}
