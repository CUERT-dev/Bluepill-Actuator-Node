#include "gp_hardware.h"
#include <string.h>

static GpResult timer_base(uint32_t clock, uint32_t hz, GpTimerPlan *out, uint32_t *period)
{
    if (!out || !period || !clock || !hz || hz > clock / 2u) return GP_BAD_ARG;
    uint64_t divisor = ((uint64_t)clock + (uint64_t)hz * 65536u - 1u) /
                       ((uint64_t)hz * 65536u);
    if (!divisor || divisor > 65536u) return GP_INVALID;
    uint64_t ticks = ((uint64_t)clock + divisor * hz - 1u) / (divisor * hz);
    if (ticks < 2u || ticks > 65536u) return GP_INVALID;
    out->prescaler = (uint16_t)(divisor - 1u);
    out->autoreload = (uint16_t)(ticks - 1u);
    out->compare = 0;
    *period = (uint32_t)ticks;
    return GP_OK;
}

GpResult GpTimer_PwmPlan(uint32_t clock, uint32_t hz, uint16_t duty, GpTimerPlan *out)
{
    uint32_t period;
    if (duty > 1000 || !out) return GP_BAD_ARG;
    GpResult result = timer_base(clock, hz, out, &period);
    if (result != GP_OK) return result;
    uint32_t compare = (uint32_t)((uint64_t)period * duty / 1000u);
    /* CCR is 16 bits: full duty at ARR=65535 is deliberately under one tick. */
    out->compare = (uint16_t)(compare > UINT16_MAX ? UINT16_MAX : compare);
    return GP_OK;
}

GpResult GpTimer_StepPlan(uint32_t clock, uint32_t hz, uint16_t high,
                          uint16_t low, GpTimerPlan *out)
{
    uint32_t period;
    if (!high || !low || !out) return GP_BAD_ARG;
    GpResult result = timer_base(clock, hz, out, &period);
    if (result != GP_OK) return result;
    uint64_t tick_us_denominator = ((uint64_t)out->prescaler + 1u) * 1000000u;
    uint64_t high_ticks = ((uint64_t)clock * high + tick_us_denominator - 1u) / tick_us_denominator;
    uint64_t low_ticks = ((uint64_t)clock * low + tick_us_denominator - 1u) / tick_us_denominator;
    if (!high_ticks || high_ticks >= period || high_ticks > UINT16_MAX ||
        high_ticks + low_ticks > period) return GP_INVALID;
    out->compare = (uint16_t)high_ticks;
    return GP_OK;
}

bool GpCanTiming_Valid(uint32_t clock, uint32_t bitrate, uint16_t prescaler,
                       uint8_t bs1, uint8_t bs2, uint8_t sjw)
{
    return clock && bitrate && prescaler >= 1 && prescaler <= 1024 &&
           bs1 >= 1 && bs1 <= 16 && bs2 >= 1 && bs2 <= 8 && sjw >= 1 &&
           sjw <= 4 && sjw <= bs2 &&
           (uint64_t)bitrate * prescaler * (1u + bs1 + bs2) == clock;
}

GpResult GpMotorHw_Init(GpMotorHw *hw, const GpMotorHwConfig *config, const GpMotorHwPort *port)
{
    if (!hw || !config || !port || !port->now_us || !port->off ||
        !port->direction || !port->apply || !port->permitted) return GP_BAD_ARG;
    GpMotorHwConfig c = *config;
    GpMotorHwPort p = *port;
    memset(hw, 0, sizeof(*hw));
    hw->port = p;
    p.off(p.user);
    GpTimerPlan probe;
    if (!c.approved || !c.duty_cap_pm || c.duty_cap_pm > 1000 ||
        !c.max_step_hz || !c.dir_setup_us ||
        GpTimer_PwmPlan(c.timer_clock_hz, c.pwm_hz, c.duty_cap_pm, &probe) != GP_OK ||
        GpTimer_StepPlan(c.timer_clock_hz, c.max_step_hz, c.pulse_us, c.low_us, &probe) != GP_OK ||
        GpTimer_StepPlan(c.timer_clock_hz, 1u, c.pulse_us, c.low_us, &probe) != GP_OK) return GP_INVALID;
    hw->config = c;
    hw->initialized = true;
    return GP_OK;
}

void GpMotorHw_Off(void *user)
{
    GpMotorHw *hw = user;
    if (!hw) return;
    if (hw->port.off) hw->port.off(hw->port.user);
    hw->pending = hw->stepping = hw->preparing = false;
    hw->direction = 0;
    hw->setup_since_us = 0;
}

GpResult GpMotorHw_Write(void *user, const MotorOutput *output)
{
    GpMotorHw *hw = user;
    if (!hw) return GP_BAD_ARG;
    if (!output || !hw->initialized || !hw->port.permitted(hw->port.user)) {
        GpMotorHw_Off(hw);
        return output ? GP_INVALID : GP_BAD_ARG;
    }
    MotorOutput request = *output;
    if (request.throttle_duty_pm < 0 || request.throttle_duty_pm > 1000 ||
        request.steering_rate_pm < -1000 || request.steering_rate_pm > 1000) {
        GpMotorHw_Off(hw);
        return GP_INVALID;
    }
    if (request.throttle_duty_pm == 0 && request.steering_rate_pm == 0) {
        GpMotorHw_Off(hw);
        return GP_OK; /* neutral must never release a hardware inhibit */
    }
    int8_t sign = request.steering_rate_pm > 0 ? 1 : request.steering_rate_pm < 0 ? -1 : 0;
    if (!sign) {
        if (hw->preparing) hw->port.off(hw->port.user);
        hw->preparing = false;
        hw->pending = false;
    } else if (sign != hw->direction || (!hw->stepping && !hw->pending && !hw->preparing)) {
        hw->port.off(hw->port.user);
        hw->direction = sign;
        hw->preparing = true;
        hw->pending = false;
    }
    if (hw->preparing) {
        GpResult ready = hw->port.prepare ? hw->port.prepare(hw->port.user) : GP_OK;
        if (ready == GP_PENDING) return GP_PENDING;
        if (ready != GP_OK) {
            GpMotorHw_Off(hw);
            return ready;
        }
        if (!hw->port.direction(hw->port.user, sign)) {
            GpMotorHw_Off(hw);
            return GP_IO_ERROR;
        }
        hw->preparing = false;
        hw->setup_since_us = hw->port.now_us(hw->port.user);
        hw->pending = true;
    }
    if (hw->pending) {
        uint32_t wait = hw->config.dir_setup_us > hw->config.enable_setup_us ?
                        hw->config.dir_setup_us : hw->config.enable_setup_us;
        if ((uint32_t)(hw->port.now_us(hw->port.user) - hw->setup_since_us) < wait) {
            return GP_PENDING;
        }
    }
    uint16_t duty = (uint16_t)request.throttle_duty_pm;
    if (duty > hw->config.duty_cap_pm) duty = hw->config.duty_cap_pm;
    GpTimerPlan pwm = {0}, step = {0};
    GpResult result = GpTimer_PwmPlan(hw->config.timer_clock_hz, hw->config.pwm_hz, duty, &pwm);
    if (sign && result == GP_OK) {
        uint32_t magnitude = (uint32_t)(sign > 0 ? request.steering_rate_pm : -request.steering_rate_pm);
        /* Ceiling quantization, in Hz, stays within the approved maximum. */
        uint32_t hz = (uint32_t)(((uint64_t)hw->config.max_step_hz * magnitude + 999u) / 1000u);
        result = GpTimer_StepPlan(hw->config.timer_clock_hz, hz, hw->config.pulse_us,
                                  hw->config.low_us, &step);
    }
    if (result != GP_OK || !hw->port.apply(hw->port.user, &pwm, sign ? &step : NULL)) {
        GpMotorHw_Off(hw);
        return result != GP_OK ? result : GP_IO_ERROR;
    }
    hw->pending = false;
    hw->stepping = sign != 0;
    return GP_OK;
}

bool GpTaskWatchdog_Ready(const GpTaskWatchdog *watchdog, uint32_t now)
{
    return watchdog && watchdog->deadline_ms && watchdog->deadline_ms < 0x80000000u &&
           watchdog->motor_seen && watchdog->can_seen &&
           (uint32_t)(now - watchdog->motor_ms) < watchdog->deadline_ms &&
           (uint32_t)(now - watchdog->can_ms) < watchdog->deadline_ms;
}

uint32_t GpSchedule_Wait(uint32_t now, uint32_t period, uint32_t *deadline)
{
    if (!deadline || !period || period >= 0x80000000u) return 0;
    if ((int32_t)(now - *deadline) >= 0) {
        /* One bounded arithmetic step, never a catch-up actuator loop. */
        *deadline += ((now - *deadline) / period + 1u) * period;
    }
    return *deadline - now;
}
