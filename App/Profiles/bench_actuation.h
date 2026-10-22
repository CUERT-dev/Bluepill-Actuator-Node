#ifndef GP_BENCH_ACTUATION_PROFILE_H
#define GP_BENCH_ACTUATION_PROFILE_H
/* Real application, actuation enabled, physically unverified commissioning
 * candidate. This profile is distinct from unpowered SignalTest. */
#if !GP_CONTROLLED_BENCH_ACTUATION || GP_UNPOWERED_SIGNAL_TEST
#error "BenchActuation requires controlled bench mode, separate from SignalTest"
#endif
#include "commissioning_parameters.h"
#endif
