/*
 * motor_driver.h -- Boda: motor driver interface (API revision 10)
 * BTS7960 throttle (forward only) + EM882S steering (STEP/DIR).
 * Hardware is reached only through config->write() and config->off().
 *
 * Units: *_pm = per-mille (1000 = 100 %), *_ms = milliseconds, *_us = microseconds.
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "gp_types.h"

/*
 * Adapter contract (no pin/timer implementation in this library):
 * - off() synchronously stops both motors and cancels any pending write.
 * - write() must stop STEP before changing DIR, establish DIR/ENA, then
 *   enforce dir_setup_us and enable_setup_us before starting STEP.
 * - GP_PENDING means BOTH motor outputs are inactive while the adapter
 *   establishes signals / waits. Retry with the latest request; a changed
 *   request must replace the pending request, never replay stale movement.
 *   Pending work must not activate outputs autonomously between calls.
 * - GP_OK means the requested output is active. Other results cause off().
 * The driver enforces reversal deadtime BEFORE write(). Setup timing is
 * delegated to the adapter: waiting before write() cannot enforce it.
 * The adapter must receive the same configuration via user and enforce
 * max_step_hz, step_pulse_us and setup intervals; write receives per-mille,
 * not Hz. No hardware timing compliance is established by host tests.
 *
 * Init: copies config and calls its off() before numerical validation.
 * Call Stop before reinitializing a live instance. A failed Init leaves the
 * instance uninitialized. Config may refer to driver->config.
 *   GP_OK, GP_BAD_ARG (NULL pointer / callback), GP_INVALID (bad number).
 */
GpResult MotorDriver_Init(MotorDriver *driver, const MotorDriverConfig *config);

/*
 * Apply: call every actuator tick. 'applied' is always written
 * (zero on stop, error or pending).
 *   GP_OK        output written, or safety forced a stop
 *   GP_PENDING   deadtime or adapter setup waiting; both outputs inactive
 *   GP_INVALID   request out of range, outputs off
 *   GP_IO_ERROR  write() failed, outputs off
 *   GP_BAD_ARG   NULL argument or driver not initialized
 * Startup and restart after Stop conservatively wait a full deadtime from
 * the first nonzero steering request (Stop has no timestamp).
 * Use a monotonic wrapping uint32_t millisecond clock; service transitions
 * within 2^31 ms. request and applied may alias. Calls must be serialized.
 */
GpResult MotorDriver_Apply(MotorDriver *driver, const MotorOutput *request,
                           const MotorSafety *safety, uint32_t now_ms,
                           MotorOutput *applied);

/* Stop: immediate off, clears BOTH outputs and all transition history,
 * preserves configuration and initialized state. NULL-safe and repeatable.
 * Movement requires a fresh Apply request and safety permission. */
void MotorDriver_Stop(MotorDriver *driver);


#endif /* MOTOR_DRIVER_H */
