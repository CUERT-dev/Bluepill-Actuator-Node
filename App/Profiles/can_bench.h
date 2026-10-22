#ifndef GP_CAN_BENCH_H
#define GP_CAN_BENCH_H
/* Gateway's 500 kbit/s operating specification, 36 MHz APB1:
 * 36 MHz / (4 * (1 + 15 + 2)) = 500 kbit/s, sample point 16/18.
 * This header approves CAN timing ONLY, never power-stage operation. */
#include "can_500k.h"
#if GP_ENABLE_ACTUATION || GP_ENABLE_WATCHDOG
#error "CAN bench profile cannot enable motors or watchdog"
#endif
#endif
