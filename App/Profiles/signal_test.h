#ifndef GP_SIGNAL_TEST_PROFILE_H
#define GP_SIGNAL_TEST_PROFILE_H
#include "can_500k.h"

/* Scope/logic-analyzer values ONLY. Disconnect motor energy before running.
 * These polarities/routes and limits do not approve mechanical forward motion,
 * installed driver readiness, reset behavior or powered operation. */
#if !GP_ENABLE_ACTUATION || !GP_UNPOWERED_SIGNAL_TEST || GP_ENABLE_WATCHDOG
#error "SignalTest requires unpowered waveform mode, CAN, and watchdog OFF"
#endif
#define GP_OUTPUT_SETTINGS_CONFIGURED 1
#define GP_THROTTLE_FORWARD_ROUTE 1u /* PB3 L-PWM; mechanical direction unknown */
#define GP_PWM_ACTIVE_HIGH 1
#define GP_REVERSE_INACTIVE_HIGH 0
#define GP_STEP_ACTIVE_HIGH 1
#define GP_DIR_POSITIVE_HIGH 1
#define GP_THROTTLE_PWM_HZ 20000u
#define GP_THROTTLE_CAP_PM 100u
#define GP_STEERING_CAP_PM 100u
#define GP_THROTTLE_SLEW_PM_S 1000u
#define GP_STEERING_SLEW_PM_S 1000u
#define GP_MAX_STEP_HZ 1000u /* 100 pm steering cap yields at most 100 Hz */
#define GP_STEP_PULSE_US 5u
#define GP_STEP_LOW_US 5u
#define GP_DIR_SETUP_US 5u
#define GP_ENABLE_SETUP_US 0u /* No ENA signal exists; no enable claim. */
#define GP_REVERSAL_DEADTIME_MS 10u
#endif
