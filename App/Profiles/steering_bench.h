#ifndef GP_STEERING_BENCH_PROFILE_H
#define GP_STEERING_BENCH_PROFILE_H
#include "bench_actuation.h"
/* Low-speed diagnostic for the reported 86CM85 vibration. This is a test
 * setting, not a rated mechanical limit: RPM still depends on drive microsteps.
 * Full signed demand maps to +/-1000 pulses/s; existing 200 pm/s slew gives
 * 200 pulses/s per second. Keep the proven throttle settings and pulse timing. */
#undef GP_MAX_STEP_HZ
#define GP_MAX_STEP_HZ 1000u
#endif
