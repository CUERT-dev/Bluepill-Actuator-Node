/* Real codec -> supervisor -> mapper -> Boda driver -> timer planner.
 * Only time, task/ISR serialization and the electrical port are simulated.
 * These values are test fixtures, never board approvals or firmware providers. */
#include "gp_control.h"
#include "gp_hardware.h"
#ifdef GP_TEST_STEERING_BENCH
#include "../../App/Profiles/steering_bench.h"
#else
#include "../../App/Profiles/commissioning_parameters.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)
#define FLAGS (GP_FLAG_VALID | GP_FLAG_ARM | GP_FLAG_DEADMAN)
typedef struct {
    GpControl control;
    GpCanRxTask rx;
    GpMotorTask motor;
    GpMotorHw hw;
    uint32_t ms, prepare_since, prepare_delay;
    uint16_t sequence;
    unsigned depth, off_count, dir_count;
    bool permitted, preparing, fail_apply;
    int8_t direction;
    GpTimerPlan pwm, step;
} Fixture;
static uint32_t enter(void *u, bool isr)
{ (void)isr; return ((Fixture *)u)->depth++; }
static void leave(void *u, bool isr, uint32_t saved)
{ (void)isr; Fixture *f = u; CHECK(f->depth == saved + 1); f->depth = saved; }
static uint32_t now_ms(void *u) { return ((Fixture *)u)->ms; }
static uint32_t now_us(void *u) { return ((Fixture *)u)->ms * 1000u; }
static void notify(void *u, GpWakeTarget target, bool isr)
{ (void)u; (void)target; (void)isr; }
static bool inputs(void *u, GpControlInputs *out)
{
    Fixture *f = u;
    *out = (GpControlInputs){.estop_ok = false, .can_bus_ok = true,
        .outputs_approved = f->permitted, .estop_mode = GP_ESTOP_NOT_CONNECTED};
    return true;
}
static bool discard(void *u)
{ Fixture *f = u; CHECK(!f->pwm.compare && !f->step.compare); return true; }
static void off(void *u)
{
    Fixture *f = u;
    ++f->off_count;
    f->pwm = f->step = (GpTimerPlan){0};
    f->preparing = false;
}
static bool permitted(void *u) { return ((Fixture *)u)->permitted; }
static GpResult prepare(void *u)
{
    Fixture *f = u;
    CHECK(!f->pwm.compare && !f->step.compare);
    if (!f->preparing) { f->preparing = true; f->prepare_since = f->ms; }
    return f->ms - f->prepare_since < f->prepare_delay ? GP_PENDING : GP_OK;
}
static bool direction(void *u, int8_t sign)
{
    Fixture *f = u;
    CHECK(!f->pwm.compare && !f->step.compare);
    f->direction = sign;
    ++f->dir_count;
    return true;
}
static bool apply(void *u, const GpTimerPlan *pwm, const GpTimerPlan *step)
{
    Fixture *f = u;
    CHECK(f->depth && f->permitted && f->motor.state == GP_ARMED);
    CHECK(!f->control.urgent_reasons && !f->control.raw_count && !f->control.event_count);
    if (f->fail_apply) return false;
    f->pwm = *pwm;
    f->step = step ? *step : (GpTimerPlan){0};
    return true;
}
static void init(Fixture *f)
{
    memset(f, 0, sizeof(*f));
    f->permitted = true;
    GpControlConfig cfg = GpControl_DefaultConfig();
    GpControlPort port = {f, enter, leave, now_ms, notify, inputs, discard};
    CHECK(GpControl_Init(&f->control, &cfg, &port) == GP_OK);
    CHECK(GpCanRx_Init(&f->rx, &f->control, GP_DecodeCan) == GP_OK);
    GpMotorHwConfig hwcfg = {72000000, 20000, 20000, 600, 5, 5, 5, 0, true};
    GpMotorHwPort hwport = {f, now_us, off, direction, apply, permitted, prepare};
    CHECK(GpMotorHw_Init(&f->hw, &hwcfg, &hwport) == GP_OK);
    CommandMapConfig map = {600, 1000, 1000, 1000, 40};
    MotorDriverConfig driver = {GpMotorHw_Write, GpMotorHw_Off, &f->hw,
                                600, 20000, 5, 5, 0, 10};
    CHECK(GpMotor_Init(&f->motor, &f->control, &map, &driver) == GP_OK);
}
static GpResult command(Fixture *f, uint32_t ms, uint16_t throttle, int16_t steering, uint8_t flags)
{
    f->ms = ms;
    TargetSample sample = {++f->sequence, throttle, steering, flags};
    GpCanFrame frame;
    CHECK(GP_EncodeCan(&sample, &frame) == GP_OK);
    CHECK(GpControl_CaptureIsr(&f->control, &frame, false, false) == GP_OK);
    CHECK(GpCanRx_Run(&f->rx) == 1);
    return GpMotor_Run(&f->motor);
}
static void arm(Fixture *f)
{
    CHECK(command(f, 0, 0, 0, GP_FLAG_VALID) == GP_OK);
    CHECK(command(f, 1, 0, 0, FLAGS) == GP_OK);
    CHECK(f->motor.state == GP_ARMED);
}
static void axes(void)
{
    Fixture f; init(&f); arm(&f);
    CHECK(command(&f, 11, 500, 0, FLAGS) == GP_OK);
    CHECK(f.pwm.compare == 36 && !f.step.compare); /* 10/1000 duty */
    for (uint32_t t = 21; t <= 101; t += 10) {
        GpResult r = command(&f, t, 500, 500, FLAGS);
        CHECK(r == GP_OK || r == GP_PENDING);
    }
    CHECK(f.pwm.compare && f.step.compare && f.direction == 1);
    GpTimerPlan step = f.step;
    unsigned offs = f.off_count;
    CHECK(command(&f, 102, 0, 500, FLAGS) == GP_OK);
    CHECK(!f.pwm.compare && f.step.autoreload == step.autoreload);
    CHECK(f.off_count == offs);
    CHECK(command(&f, 111, 500, 500, FLAGS) == GP_OK);
    GpTimerPlan pwm = f.pwm;
    CHECK(command(&f, 112, 500, 0, FLAGS) == GP_OK);
    CHECK(f.pwm.compare == pwm.compare && !f.step.compare && f.off_count == offs);
    CHECK(command(&f, 113, 0, 0, FLAGS) == GP_OK);
    CHECK(!f.pwm.compare && !f.step.compare && f.motor.state == GP_ARMED);
    CHECK(f.direction == 1); /* neutral never requests wheel centering */
}
static void readiness_stop(void)
{
    Fixture f; init(&f); arm(&f); f.prepare_delay = 200;
    for (uint32_t t = 11; t <= 111; t += 10)
        CHECK(command(&f, t, 500, -500, FLAGS) == GP_PENDING);
    CHECK(f.hw.preparing && !f.dir_count && !f.pwm.compare && !f.step.compare);
    CHECK(command(&f, 112, 0, -500, FLAGS) == GP_PENDING);
    CHECK(f.hw.preparing && f.preparing); /* throttle neutral preserves the wait */
    CHECK(command(&f, 113, 0, 0, 0) == GP_INVALID);
    CHECK(!f.hw.preparing && !f.hw.pending && !f.preparing);
    CHECK(f.motor.state == GP_WAIT_FOR_NEUTRAL);
    CHECK(command(&f, 400, 500, -500, FLAGS) == GP_OK);
    CHECK(!f.pwm.compare && !f.step.compare && !f.dir_count);
    CHECK(command(&f, 401, 0, 0, GP_FLAG_VALID) == GP_OK);
    CHECK(command(&f, 402, 500, -500, FLAGS) == GP_OK);
    for (uint32_t t = 412; t <= 642; t += 10) {
        GpResult r = command(&f, t, 500, -500, FLAGS);
        CHECK(r == GP_OK || r == GP_PENDING);
    }
    CHECK(f.pwm.compare && f.step.compare && f.direction == -1);
}
static void reversal_failure(void)
{
    Fixture f; init(&f); arm(&f);
    for (uint32_t t = 11; t <= 101; t += 10) (void)command(&f, t, 500, 30, FLAGS);
    CHECK(f.direction == 1 && f.step.compare && f.pwm.compare);
    bool pending = false;
    for (uint32_t t = 111; t <= 201; t += 10) {
        GpResult r = command(&f, t, 500, -30, FLAGS);
        if (r == GP_PENDING) { pending = true; CHECK(!f.pwm.compare && !f.step.compare); }
        else CHECK(r == GP_OK);
    }
    CHECK(pending && f.direction == -1 && f.step.compare && f.pwm.compare);
    f.fail_apply = true;
    CHECK(command(&f, 211, 500, -30, FLAGS) == GP_IO_ERROR);
    CHECK(f.motor.latched_faults & GP_FAULT_OUTPUT);
    CHECK(!f.pwm.compare && !f.step.compare);
    f.fail_apply = false;
    CHECK(command(&f, 212, 500, -30, FLAGS) == GP_INVALID);
    CHECK(GpMotor_Recover(&f.motor) == GP_OK);
    CHECK(command(&f, 213, 500, -30, FLAGS) == GP_OK);
    CHECK(f.motor.state == GP_WAIT_FOR_NEUTRAL && !f.pwm.compare && !f.step.compare);
}
static void rx_setup_progress(void)
{
    Fixture f; init(&f); arm(&f); f.prepare_delay = 200;
    f.motor.map.config.throttle_slew_pm_s = 1;
    f.motor.map.config.steering_slew_pm_s = 1;
    bool ran = false;
    for (uint32_t t = 20; t <= 3000; t += 20) {
        f.ms = t;
        TargetSample sample = {++f.sequence, 500, 500, FLAGS};
        GpCanFrame frame;
        CHECK(GP_EncodeCan(&sample, &frame) == GP_OK);
        CHECK(GpControl_CaptureIsr(&f.control, &frame, false, false) == GP_OK);
        CHECK(GpMotor_Run(&f.motor) == GP_PENDING);
        CHECK(!f.pwm.compare && !f.step.compare);
        CHECK(GpCanRx_Run(&f.rx) == 1);
        f.ms = t + 1;
        CHECK(GpMotor_Run(&f.motor) == GP_OK);
        f.ms = t + 11;
        GpResult r = GpMotor_Run(&f.motor);
        CHECK(r == GP_OK || r == GP_PENDING);
        ran |= f.step.compare != 0;
        CHECK(!f.motor.latched_faults);
    }
    CHECK(ran); /* CAN races must not continually cancel steering setup. */
    CHECK(command(&f, 3012, 0, 0, 0) == GP_INVALID);
    CHECK(!f.hw.pending && !f.hw.preparing && !f.step.compare && !f.pwm.compare);
}

static void readiness_slew(void)
{
    Fixture f; init(&f); arm(&f); f.prepare_delay = 200;
    for (uint32_t t = 11; t <= 221; t += 10) {
        CHECK(command(&f, t, 500, 500, FLAGS) == GP_PENDING);
        CHECK(!f.pwm.compare && !f.step.compare);
    }
    CHECK(command(&f, 231, 500, 500, FLAGS) == GP_OK);
    /* A 1000 pm/s rise over this 10 ms output interval permits 10 pm.
     * Time spent with both outputs suppressed must not accumulate a jump. */
    CHECK(f.motor.applied.throttle_duty_pm == 10);
    CHECK(f.motor.applied.steering_rate_pm == 10);
    CHECK(f.pwm.compare == 36);
    CHECK(command(&f, 241, 500, 500, FLAGS) == GP_OK);
    CHECK(f.motor.applied.throttle_duty_pm == 20);
    CHECK(f.motor.applied.steering_rate_pm == 20);
}
static void readiness_fractional(void)
{
    Fixture f; init(&f); arm(&f); f.prepare_delay = 200;
    f.motor.map.config.throttle_slew_pm_s = 1;
    f.motor.map.config.steering_slew_pm_s = 1;
    for (uint32_t t = 11; t <= 1221; t += 10) {
        GpResult r = command(&f, t, 500, 500, FLAGS);
        CHECK(r == GP_OK || r == GP_PENDING);
    }
    /* Sub-unit slew progress must survive retries, so readiness can finish. */
    CHECK(f.motor.applied.throttle_duty_pm == 1);
    CHECK(f.motor.applied.steering_rate_pm == 1);
    CHECK(f.pwm.compare && f.step.compare && f.dir_count == 1);
}
static void pending_fractional_neutral(void)
{
    Fixture f; init(&f); arm(&f); f.prepare_delay = 200;
    f.motor.map.config.throttle_slew_pm_s = 1;
    f.motor.map.config.steering_slew_pm_s = 1;
    for (uint32_t t = 11; t <= 1001; t += 10) (void)command(&f, t, 500, 500, FLAGS);
    CHECK(f.motor.output_pending && f.motor.map.throttle_remainder == 990);
    CHECK(command(&f, 1002, 0, 500, FLAGS) == GP_PENDING);
    CHECK(f.motor.map.throttle_remainder == 0);
    for (uint32_t t = 1011; t <= 1221; t += 10) {
        GpResult r = command(&f, t, 0, 500, FLAGS);
        CHECK(r == GP_OK || r == GP_PENDING);
        CHECK(f.motor.map.throttle_remainder == 0);
    }
    CHECK(f.motor.applied.steering_rate_pm == 1 && !f.pwm.compare);
    CHECK(command(&f, 1231, 500, 500, FLAGS) == GP_OK);
    CHECK(f.motor.applied.throttle_duty_pm == 0 && f.motor.map.throttle_remainder == 10);
}
/* Actual candidate settings through the real production command pipeline.
 * Only registers/time are simulated; no fixture limits replace the header. */
static void commissioning_range(bool direct)
{
    Fixture f; init(&f);
    f.control.config.direct_commands = direct;
    uint8_t demand_flags = direct ? GP_FLAG_VALID : FLAGS;
    GpMotorHwConfig hw = {72000000, GP_THROTTLE_PWM_HZ, GP_MAX_STEP_HZ,
        GP_THROTTLE_CAP_PM, GP_STEP_PULSE_US, GP_STEP_LOW_US,
        GP_DIR_SETUP_US, GP_ENABLE_SETUP_US, true};
    GpMotorHwPort port = {&f, now_us, off, direction, apply, permitted, NULL};
    CHECK(GpMotorHw_Init(&f.hw, &hw, &port) == GP_OK);
    CommandMapConfig map = {GP_THROTTLE_CAP_PM, GP_STEERING_CAP_PM,
        GP_THROTTLE_SLEW_PM_S, GP_STEERING_SLEW_PM_S, 40};
    MotorDriverConfig driver = {GpMotorHw_Write, GpMotorHw_Off, &f.hw,
        GP_THROTTLE_CAP_PM, GP_MAX_STEP_HZ, GP_STEP_PULSE_US,
        GP_DIR_SETUP_US, GP_ENABLE_SETUP_US, GP_REVERSAL_DEADTIME_MS};
    CHECK(GpMotor_Init(&f.motor, &f.control, &map, &driver) == GP_OK);
    if (direct) {
        /* Nonzero first command: no neutral, ARM or DEADMAN ever supplied. */
        CHECK(command(&f, 1, 1000, 1000, demand_flags) == GP_OK);
        CHECK(f.motor.state == GP_ARMED && f.motor.latest.flags == GP_FLAG_VALID);
    } else arm(&f);
    uint32_t t;
    for (t = 11; t <= 10101; t += 10) {
        GpResult r = command(&f, t, 1000, 1000, demand_flags);
        CHECK(r == GP_OK || r == GP_PENDING);
    }
    CHECK(f.motor.applied.throttle_duty_pm == 1000);
    CHECK(f.motor.applied.steering_rate_pm == 1000 && f.direction == 1);
    CHECK(f.pwm.prescaler == 0 && f.pwm.autoreload == 3599 && f.pwm.compare == 3600);
#ifdef GP_TEST_STEERING_BENCH
    CHECK(GP_MAX_STEP_HZ == 1000 && GP_STEERING_SLEW_PM_S == 200);
    CHECK(f.step.prescaler == 1 && f.step.autoreload == 35999 && f.step.compare == 108);
#else
    CHECK(f.step.prescaler == 0 && f.step.autoreload == 432 && f.step.compare == 216);
#endif
    GpTimerPlan full_step = f.step;
    CHECK(command(&f, t, 0, 1000, demand_flags) == GP_OK); t += 10;
    CHECK(!f.pwm.compare && f.step.compare == full_step.compare); /* independent throttle neutral */
    /* Allow deceleration to zero, reversal setup, then the existing full ramp. */
    for (uint32_t end = t + 15100; t <= end; t += 10) {
        GpResult r = command(&f, t, 1000, -1000, demand_flags);
        CHECK(r == GP_OK || r == GP_PENDING);
        if (r == GP_PENDING) CHECK(!f.pwm.compare && !f.step.compare);
    }
    CHECK(f.motor.applied.throttle_duty_pm == 1000);
    CHECK(f.motor.applied.steering_rate_pm == -1000 && f.direction == -1);
    CHECK(f.pwm.compare == 3600 && f.step.compare == full_step.compare);
    CHECK(command(&f, t, 1000, 0, demand_flags) == GP_OK); t += 10;
    CHECK(f.pwm.compare == 3600 && !f.step.compare); /* independent steering neutral */
    CHECK(command(&f, t, 0, 0, 0) == GP_INVALID);
    CHECK(!f.pwm.compare && !f.step.compare && f.motor.state == GP_WAIT_FOR_NEUTRAL);

    /* Every nonzero magnitude must have a realizable plan, not only endpoints. */
    for (uint32_t pm = 1; pm <= 1000; ++pm) {
        GpTimerPlan plan;
        uint32_t hz = (GP_MAX_STEP_HZ * pm + 999u) / 1000u;
        CHECK(GpTimer_StepPlan(72000000, hz, GP_STEP_PULSE_US, GP_STEP_LOW_US, &plan) == GP_OK);
        uint64_t div = (uint64_t)plan.prescaler + 1u;
        uint64_t period = (uint64_t)plan.autoreload + 1u;
        CHECK(div * period * hz >= 72000000u);
        CHECK(div * plan.compare * 1000000u >= 72000000u * GP_STEP_PULSE_US);
        CHECK(div * (period - plan.compare) * 1000000u >= 72000000u * GP_STEP_LOW_US);
    }
    GpTimerPlan invalid;
    CHECK(GpTimer_StepPlan(72000000, 200000, GP_STEP_PULSE_US, GP_STEP_LOW_US,
                           &invalid) == GP_INVALID); /* 3+3 us cannot fit at 200 kHz */
    if (direct) {
        /* Steering alone, flags=VALID only, through real driver/timer planning.
         * A lost stream stops motion; a new steering frame restarts it without
         * recover, neutral, ARM, DEADMAN or any throttle demand. */
        t += 10;
        for (uint32_t end = t + 1000; t <= end; t += 10) {
            GpResult r = command(&f, t, 0, 500, GP_FLAG_VALID);
            CHECK(r == GP_OK || r == GP_PENDING);
            CHECK(!f.pwm.compare);
        }
        CHECK(f.motor.latest.steering_pm == 500 && f.step.compare && f.direction == 1);
        f.ms = t + 200;
        CHECK(GpMotor_Run(&f.motor) == GP_INVALID);
        CHECK(f.motor.latched_faults & GP_FAULT_TIMEOUT);
        CHECK(!f.pwm.compare && !f.step.compare);
        t = f.ms + 1;
        CHECK(command(&f, t, 0, -500, GP_FLAG_VALID) == GP_OK);
        CHECK(f.motor.latest.steering_pm == -500 && !f.motor.latched_faults);
        t += 10;
        for (uint32_t end = t + 1000; t <= end; t += 10) {
            GpResult r = command(&f, t, 0, -500, GP_FLAG_VALID);
            CHECK(r == GP_OK || r == GP_PENDING);
            CHECK(!f.pwm.compare);
        }
        CHECK(f.motor.applied.steering_rate_pm < 0 && f.step.compare && f.direction == -1);
    }
}
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "commissioning_range")) commissioning_range(false);
    else if (!strcmp(argv[1], "direct_range")) commissioning_range(true);
    else if (!strcmp(argv[1], "axes")) axes();
    else if (!strcmp(argv[1], "readiness_stop")) readiness_stop();
    else if (!strcmp(argv[1], "rx_setup_progress")) rx_setup_progress();
    else if (!strcmp(argv[1], "reversal_failure")) reversal_failure();
    else if (!strcmp(argv[1], "readiness_slew")) readiness_slew();
    else if (!strcmp(argv[1], "readiness_fractional")) readiness_fractional();
    else if (!strcmp(argv[1], "pending_fractional_neutral")) pending_fractional_neutral();
    else return 2;
    return 0;
}
