/*
 * motor_driver.c -- motor driver logic (throttle + steering).
 *
 * This file does NOT touch pins, timers or registers.
 * It decides WHAT the motors should do, then calls two functions that
 * Farah provides in the adapter:
 *   config.write(user, &output) -> apply throttle PWM, STEP rate and DIR
 *   config.off(user)            -> switch both motors off right now
 *
 * How to use:
 *   1. Call MotorDriver_Init() once at startup.
 *   2. Call MotorDriver_Apply() every actuator tick (same task every time).
 *   3. Call MotorDriver_Stop() for an immediate stop.
 */
#include "motor_driver.h"

/* Set both output values to 0. */
static void zero(MotorOutput *o)
{
    o->throttle_duty_pm = 0;
    o->steering_rate_pm = 0;
}

/*
 * Turn everything off.
 * If steering pulses were running, remember the time they stopped.
 * The reversal wait (deadtime) is counted from this time.
 */
static void halt(MotorDriver *d, uint32_t now_ms)
{
    d->config.off(d->config.user);
    if (d->previous.steering_rate_pm != 0) {
        d->zero_since_ms = now_ms;
    }
    zero(&d->previous);
}

/*
 * Check the config, copy it, and call off() so the motors start disabled.
 * Returns:
 *   GP_OK       ready to use
 *   GP_BAD_ARG  a pointer or a callback (write/off) is NULL
 *   GP_INVALID  a number is out of range
 */
GpResult MotorDriver_Init(MotorDriver *driver, const MotorDriverConfig *config)
{
    if (driver == NULL) {
        return GP_BAD_ARG;
    }
    driver->initialized = false;
    if (config == NULL || config->write == NULL || config->off == NULL) {
        return GP_BAD_ARG;
    }
    /* max_duty_pm <= 1000, max_step_hz > 0, pulse width > 0, deadtime < 2^31 */
    if (config->max_duty_pm > 1000 || config->max_step_hz == 0 || config->step_pulse_us == 0 ||
        config->reversal_deadtime_ms >= 0x80000000u) {
        return GP_INVALID;
    }

    driver->config = *config;
    zero(&driver->previous);
    driver->zero_since_ms = 0;
    driver->last_steering_direction = 0;
    config->off(config->user); /* off before anything is enabled */
    driver->initialized = true;
    return GP_OK;
}

/*
 * Call every actuator tick with the new request.
 *   request  : throttle 0..1000, steering -1000..1000 (per-mille)
 *   safety   : estop_ok and enabled must both be true, or motors stop
 *   now_ms   : current time in ms (used for the reversal wait)
 *   applied  : what was really sent to write(); always set, zero if not sent
 *
 * Returns:
 *   GP_OK        output sent (or stopped because safety said so)
 *   GP_PENDING   steering is changing direction, waiting. Motors are off.
 *                Just call Apply again next tick.
 *   GP_INVALID   request out of range. Motors are off.
 *   GP_IO_ERROR  write() failed. Motors are off.
 *   GP_BAD_ARG   NULL argument or Init was not done.
 */
GpResult MotorDriver_Apply(MotorDriver *driver, const MotorOutput *request,
                           const MotorSafety *safety, uint32_t now_ms,
                           MotorOutput *applied)
{
    MotorOutput out;
    int8_t dir;

    if (applied != NULL) {
        zero(applied);
    }
    if (driver == NULL || !driver->initialized || request == NULL || safety == NULL ||
        applied == NULL) {
        if (driver != NULL && driver->initialized) {
            halt(driver, now_ms);
        }
        return GP_BAD_ARG;
    }

    /* Safety first: e-stop or disabled means stop, every call. */
    if (!safety->estop_ok || !safety->enabled) {
        halt(driver, now_ms);
        return GP_OK;
    }

    /* Reject bad demand: throttle must be 0..1000, steering -1000..1000. */
    if (request->throttle_duty_pm < 0 || request->throttle_duty_pm > 1000 ||
        request->steering_rate_pm < -1000 || request->steering_rate_pm > 1000) {
        halt(driver, now_ms);
        return GP_INVALID;
    }

    /* Limit throttle to max_duty_pm.
     * Steering sign = DIR (+1 or -1). Steering 0 = no pulses. */
    out = *request;
    if (out.throttle_duty_pm > (int16_t)driver->config.max_duty_pm) {
        out.throttle_duty_pm = (int16_t)driver->config.max_duty_pm;
    }
    dir = (out.steering_rate_pm > 0) ? 1 : (out.steering_rate_pm < 0 ? -1 : 0);

    /*
     * Steering changes direction (left <-> right):
     * stop the pulses, wait reversal_deadtime_ms plus the DIR and enable
     * setup times, and only then send the new direction.
     * While waiting: motors off, return GP_PENDING.
     */
    if (dir != 0 && driver->last_steering_direction != 0 &&
        dir != driver->last_steering_direction) {
        /* Total wait in ms. The us setup times are rounded up to whole ms. */
        uint32_t hold = driver->config.reversal_deadtime_ms +
                        ((uint32_t)driver->config.dir_setup_us + 999u) / 1000u +
                        ((uint32_t)driver->config.enable_setup_us + 999u) / 1000u;
        /* Pulses still running, or not enough time passed since they stopped. */
        if (driver->previous.steering_rate_pm != 0 ||
            (uint32_t)(now_ms - driver->zero_since_ms) < hold) {
            halt(driver, now_ms);
            return GP_PENDING;
        }
    }

    /* Send to hardware. If it fails, turn everything off. */
    if (driver->config.write(driver->config.user, &out) != GP_OK) {
        halt(driver, now_ms);
        return GP_IO_ERROR;
    }

    /* Steering just went to zero: remember when, for the reversal wait. */
    if (out.steering_rate_pm == 0 && driver->previous.steering_rate_pm != 0) {
        driver->zero_since_ms = now_ms;
    }
    /* Update the last direction only after a successful write. */
    if (dir != 0) {
        driver->last_steering_direction = dir;
    }
    driver->previous = out;
    *applied = out;
    return GP_OK;
}

/*
 * Immediate stop: calls off() and zeroes throttle. Safe to call with NULL.
 * Config is kept.
 */
void MotorDriver_Stop(MotorDriver *d)
{
    if (d == NULL || !d->initialized) {
        return;
    }
    d->config.off(d->config.user);
    /* Keep previous.steering_rate_pm: Stop has no now_ms, so the next Apply
     * must see "pulses were running" and stamp zero_since_ms via halt(). */
    d->previous.throttle_duty_pm = 0;
}