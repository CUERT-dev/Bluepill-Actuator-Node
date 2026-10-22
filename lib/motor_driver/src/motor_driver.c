/*
 * Motor policy only: no pins, timers, tasks or hardware adapters.
 * See motor_driver.h for the callback timing and cancellation contract.
 */
#include "motor_driver.h"

#include <string.h>

static void zero(MotorOutput *output)
{
    if (output != NULL) {
        output->throttle_duty_pm = 0;
        output->steering_rate_pm = 0;
    }
}

/* Keep direction history only during an ordinary reversal/deceleration. */
static void halt(MotorDriver *driver, uint32_t now_ms)
{
    driver->config.off(driver->config.user);
    driver->zero_since_ms = now_ms;
    zero(&driver->previous);
}

void MotorDriver_Stop(MotorDriver *driver)
{
    if (driver == NULL) {
        return;
    }
    if (driver->initialized) {
        driver->config.off(driver->config.user);
    }
    zero(&driver->previous);
    driver->zero_since_ms = 0;
    driver->last_steering_direction = 0;
}

GpResult MotorDriver_Init(MotorDriver *driver, const MotorDriverConfig *config)
{
    MotorDriverConfig copy;

    if (driver == NULL) {
        return GP_BAD_ARG;
    }
    if (config == NULL) {
        memset(driver, 0, sizeof(*driver));
        return GP_BAD_ARG;
    }
    copy = *config;
    memset(driver, 0, sizeof(*driver));
    if (copy.off != NULL) {
        copy.off(copy.user);
    }
    if (copy.write == NULL || copy.off == NULL) {
        return GP_BAD_ARG;
    }
    /* A STEP period must contain the configured high pulse and a low phase.
     * Device-specific minimum high/low times remain adapter requirements. */
    if (copy.max_duty_pm > 1000 || copy.max_step_hz == 0 ||
        copy.step_pulse_us == 0 ||
        (uint64_t)copy.max_step_hz * copy.step_pulse_us >= 1000000u ||
        copy.reversal_deadtime_ms >= 0x80000000u) {
        return GP_INVALID;
    }
    driver->config = copy;
    driver->initialized = true;
    return GP_OK;
}

GpResult MotorDriver_Apply(MotorDriver *driver, const MotorOutput *request,
                           const MotorSafety *safety, uint32_t now_ms,
                           MotorOutput *applied)
{
    MotorOutput out;
    GpResult result;
    int8_t dir;

    if (driver == NULL || !driver->initialized || request == NULL ||
        safety == NULL || applied == NULL) {
        MotorDriver_Stop(driver);
        zero(applied);
        return GP_BAD_ARG;
    }
    /* Copy before writing applied, including request == applied. */
    out = *request;
    if (!safety->estop_ok || !safety->enabled) {
        MotorDriver_Stop(driver);
        zero(applied);
        return GP_OK;
    }
    if (out.throttle_duty_pm < 0 || out.throttle_duty_pm > 1000 ||
        out.steering_rate_pm < -1000 || out.steering_rate_pm > 1000) {
        MotorDriver_Stop(driver);
        zero(applied);
        return GP_INVALID;
    }
    if (out.throttle_duty_pm == 0 && out.steering_rate_pm == 0) {
        MotorDriver_Stop(driver);
        zero(applied);
        return GP_OK;
    }
    if (out.throttle_duty_pm > (int16_t)driver->config.max_duty_pm) {
        out.throttle_duty_pm = (int16_t)driver->config.max_duty_pm;
    }
    dir = (out.steering_rate_pm > 0) ? 1 : (out.steering_rate_pm < 0 ? -1 : 0);

    if (dir != 0) {
        if (driver->last_steering_direction == 0 ||
            (driver->previous.steering_rate_pm != 0 &&
             dir != driver->last_steering_direction)) {
            /* Stop STEP first. No DIR change is requested until deadtime ends. */
            halt(driver, now_ms);
            driver->last_steering_direction = dir;
        }
        if (driver->previous.steering_rate_pm == 0 &&
            (uint32_t)(now_ms - driver->zero_since_ms) <
                driver->config.reversal_deadtime_ms) {
            /* A throttle-only command may have run since steering stopped. */
            if (driver->previous.throttle_duty_pm != 0) {
                driver->config.off(driver->config.user);
                zero(&driver->previous);
            }
            zero(applied);
            return GP_PENDING;
        }
    }

    result = driver->config.write(driver->config.user, &out);
    if (result == GP_PENDING) {
        /* The adapter guarantees inactive outputs and retains its setup
         * phase. Calling off here would cancel that phase on every retry. */
        if (driver->previous.steering_rate_pm != 0) {
            driver->zero_since_ms = now_ms;
        }
        zero(&driver->previous);
        zero(applied);
        return GP_PENDING;
    }
    if (result != GP_OK) {
        MotorDriver_Stop(driver);
        zero(applied);
        return GP_IO_ERROR;
    }
    if (dir == 0 && driver->previous.steering_rate_pm != 0) {
        driver->zero_since_ms = now_ms;
    }
    if (dir != 0) {
        driver->last_steering_direction = dir;
    }
    driver->previous = out;
    *applied = out;
    return GP_OK;
}
