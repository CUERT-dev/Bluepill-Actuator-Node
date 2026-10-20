/*
 * motor_driver.h -- Boda: motor driver interface (API revision 10)
 * BTS7960 throttle (forward only) + EM882S steering (STEP/DIR).
 * Hardware is reached only through config->write() and config->off().
 *
 * Units: *_pm = per-mille (1000 = 100 %), *_ms = milliseconds, *_us = microseconds.
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//our typedefs

typedef enum {
    GP_OK = 0,
    GP_PENDING,
    GP_BAD_ARG,
    GP_INVALID,
    GP_STALE,
    GP_IO_ERROR,
    GP_NO_SPACE
} GpResult;


typedef struct {
    int16_t throttle_duty_pm, steering_rate_pm;
} MotorOutput;

typedef struct {
    bool estop_ok, enabled;
} MotorSafety;

typedef GpResult (*MotorWriteFn)(void *user, const MotorOutput *output);

typedef void (*MotorOffFn)(void *user);

typedef struct {
    MotorWriteFn write;
    MotorOffFn off;
    void *user;
    uint16_t max_duty_pm;
    uint32_t max_step_hz;
    uint16_t step_pulse_us, dir_setup_us, enable_setup_us;
    uint32_t reversal_deadtime_ms;
} MotorDriverConfig;

typedef struct {
    MotorDriverConfig config;
    MotorOutput previous;
    uint32_t zero_since_ms;
    int8_t last_steering_direction;
    bool initialized;
} MotorDriver;


/*
 * Init: validates and copies config, calls off() first.
 *   GP_OK, GP_BAD_ARG (NULL pointer / callback), GP_INVALID (bad number).
 */
GpResult MotorDriver_Init(MotorDriver *driver, const MotorDriverConfig *config);

/*
 * Apply: call every actuator tick. 'applied' is always written
 * (zero on stop, error or pending).
 *   GP_OK        output written, or safety forced a stop
 *   GP_PENDING   steering reversal waiting, outputs off, retry next tick
 *   GP_INVALID   request out of range, outputs off
 *   GP_IO_ERROR  write() failed, outputs off
 *   GP_BAD_ARG   NULL argument or driver not initialized
 */
GpResult MotorDriver_Apply(MotorDriver *driver, const MotorOutput *request,
                           const MotorSafety *safety, uint32_t now_ms,
                           MotorOutput *applied);

/* Stop: immediate off, zeroes throttle, keeps config. Steering history is kept
 * so the next Apply restarts the reversal deadtime. NULL-safe. */
void MotorDriver_Stop(MotorDriver *driver);


#endif /* MOTOR_DRIVER_H */