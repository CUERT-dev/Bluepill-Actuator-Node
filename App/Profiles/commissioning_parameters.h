#ifndef GP_COMMISSIONING_PARAMETERS_H
#define GP_COMMISSIONING_PARAMETERS_H
/* Commissioning candidate: numerical settings, NOT physical approval.
 * Intended wiring: docs/hardware/final_pinout.png. Revision-10 demands are
 * permille, with independent throttle duty and signed steering pulse rate.
 * See docs/COMMISSIONING_CANDIDATE.md for provenance and unknowns. */
#include "can_500k.h" /* Established 500 kbit/s, 36 MHz APB1, 4/15/2/1. */

#define GP_OUTPUT_SETTINGS_CONFIGURED 1
#define GP_BENCH_COMMISSIONING_APPROVED 0 /* Physically unverified. */
#define GP_STEERING_EXTERNAL_READY_BEFORE_ARM 0 /* No readiness readback. */

/* Intended route is fixed. Signs/logic levels are provisional until measured:
 * L-PWM does not establish mechanical forward; DIR high is logical positive. */
#define GP_THROTTLE_FORWARD_ROUTE 1u /* PB3 L-PWM/TIM2_CH2; PB4 R-PWM inactive */
#define GP_PWM_ACTIVE_HIGH 1
#define GP_REVERSE_INACTIVE_HIGH 0
#define GP_STEP_ACTIVE_HIGH 1 /* PUL+ through level shifter, PUL- at GND */
#define GP_DIR_POSITIVE_HIGH 1

/* Full revision-10 command range. No commissioning amplitude/rate clamp.
 * 20 kHz is the existing BTS7960 setting; 1000 permille is 100% duty.
 * Slew rates are unchanged provisional settings, not steady-state caps. */
#define GP_THROTTLE_PWM_HZ 20000u
#define GP_THROTTLE_CAP_PM 1000u
#define GP_THROTTLE_SLEW_PM_S 100u   /* 0 -> full duty in 10 s, excluding setup */
#define GP_STEERING_CAP_PM 1000u
#define GP_STEERING_SLEW_PM_S 200u   /* 0 -> full magnitude in 5 s */

/* Leadshine EM882S manual sections 2.1, 3.1 and 10:
 * <=200 kHz, STEP high/low >=2.5 us, DIR setup >=5 us.
 * Existing integer-us interface rounds 2.5 upward to 3; no sub-us redesign.
 * The maximum whole-Hz request fitting both phases is floor(1e6/(3+3)).
 * At 72 MHz, existing ceil-period planning gives PSC=0, ARR=432, CCR=216:
 * 166281.755 STEP/s at full command, high=3 us, low=3.013889 us.
 * This is a signal-interface limit, NOT a demonstrated motor/mechanism speed. */
#define GP_STEP_PULSE_US 3u
#define GP_STEP_LOW_US 3u
#define GP_MAX_STEP_HZ (1000000u / (GP_STEP_PULSE_US + GP_STEP_LOW_US))
#define GP_DIR_SETUP_US 5u
#define GP_REVERSAL_DEADTIME_MS 10u
#define GP_ENABLE_SETUP_US 0u /* ENA is unwired; no enable timing/control claim. */
#endif
