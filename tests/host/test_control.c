#include "gp_control.h"
#include "gp_board_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)
#define FLAGS (GP_FLAG_VALID | GP_FLAG_ARM | GP_FLAG_DEADMAN)

enum Context { TEST, ISR, RX, MOTOR };
typedef struct Fixture Fixture;
struct Fixture {
    GpControl control;
    GpCanRxTask rx;
    GpMotorTask motor;
    uint32_t now;
    GpControlInputs inputs;
    MotorOutput physical;
    unsigned lock_depth, off_count, writes, nonzero_writes, decode_calls;
    unsigned notifications[2], isr_notifications;
    enum Context context;
    bool allow_motion, inputs_read_ok, auto_motor_notify, discard_ok, production_codec;
    unsigned fifo_buffered;
    GpResult driver_result;
    TargetSample samples[512];
    GpResult decode_results[512];
    unsigned sample_count;
    void (*decode_hook)(Fixture *);
    void (*unlocked_hook)(Fixture *);
};
static Fixture *current;

static bool nonzero(MotorOutput output)
{
    return output.throttle_duty_pm != 0 || output.steering_rate_pm != 0;
}

static uint32_t enter(void *user, bool isr)
{
    Fixture *f = user;
    CHECK(isr == (f->context == ISR));
    return f->lock_depth++;
}

static void leave(void *user, bool isr, uint32_t saved)
{
    Fixture *f = user;
    CHECK(isr == (f->context == ISR));
    CHECK(f->lock_depth == saved + 1);
    f->lock_depth = saved;
    if (saved == 0 && f->unlocked_hook != NULL && f->context == MOTOR) {
        void (*hook)(Fixture *) = f->unlocked_hook;
        f->unlocked_hook = NULL;
        hook(f);
    }
}

static uint32_t clock_ms(void *user)
{
    return ((Fixture *)user)->now;
}

static GpResult motor_run(Fixture *f, uint32_t now)
{
    enum Context old = f->context;
    f->context = MOTOR;
    f->now = now;
    GpResult result = GpMotor_Run(&f->motor);
    f->context = old;
    CHECK(f->lock_depth == 0);
    return result;
}

static void notify(void *user, GpWakeTarget target, bool isr)
{
    Fixture *f = user;
    CHECK(f->lock_depth == 0);
    CHECK(isr == (f->context == ISR));
    ++f->notifications[target];
    if (isr) ++f->isr_notifications;
    if (f->auto_motor_notify && target == GP_WAKE_MOTOR) {
        (void)motor_run(f, f->now);
    }
}

static bool read_inputs(void *user, GpControlInputs *inputs)
{
    Fixture *f = user;
    CHECK(f->context == MOTOR && f->lock_depth != 0);
    *inputs = f->inputs;
    return f->inputs_read_ok;
}

static bool discard_rx(void *user)
{
    Fixture *f = user;
    CHECK(f->context == MOTOR && f->lock_depth != 0);
    CHECK(!nonzero(f->physical)); /* hardware fence MUST follow output off */
    if (f->discard_ok) {
        f->fifo_buffered = 0;
        f->inputs.hardware_rx_pending = false;
    }
    return f->discard_ok;
}

static void off(void *user)
{
    Fixture *f = user;
    CHECK(f->context == MOTOR && f->lock_depth != 0);
    ++f->off_count;
    f->physical = (MotorOutput){0, 0};
}

static GpResult write_motor(void *user, const MotorOutput *output)
{
    Fixture *f = user;
    CHECK(f->context == MOTOR && f->lock_depth != 0);
    ++f->writes;
    if (nonzero(*output)) {
        /* Test oracle at EVERY attempted nonzero write, not just final state.
         * allow_motion is independent permission supplied by each scenario. */
        CHECK(f->allow_motion);
        CHECK(f->inputs.estop_mode == GP_ESTOP_NOT_CONNECTED || f->inputs.estop_ok);
        CHECK(f->inputs.can_bus_ok && f->inputs.outputs_approved);
        CHECK(!f->inputs.hardware_rx_pending);
        CHECK(f->control.urgent_reasons == 0);
        CHECK(f->control.raw_count == 0 && f->control.event_count == 0);
        CHECK(!f->control.rx_inflight ||
              f->control.inflight_generation != f->control.generation);
        CHECK(f->motor.state == GP_ARMED && f->motor.latched_faults == 0);
        uint8_t required = f->control.config.direct_commands ? GP_FLAG_VALID : FLAGS;
        CHECK((f->motor.latest.flags & required) == required);
        CHECK((uint32_t)(f->now - f->motor.latest_received_ms) <
              f->control.config.command_timeout_ms);
        ++f->nonzero_writes;
    }
    f->physical = f->driver_result == GP_PENDING ? (MotorOutput){0, 0} : *output;
    return f->driver_result;
}

/* A token-indexed test double, NOT a proposed CAN wire encoding.
 * Payload contains only an index into this fixture's TargetSample array. */
static GpResult decode(const GpCanFrame *frame, TargetSample *sample)
{
    Fixture *f = current;
    CHECK(f->context == RX && f->lock_depth == 0);
    ++f->decode_calls;
    unsigned index = frame->data[0] | ((unsigned)frame->data[1] << 8);
    CHECK(index < f->sample_count);
    if (f->decode_hook != NULL) {
        void (*hook)(Fixture *) = f->decode_hook;
        f->decode_hook = NULL;
        hook(f);
    }
    *sample = f->samples[index];
    return frame->dlc == 8 ? f->decode_results[index] : GP_INVALID;
}

static void init_with(Fixture *f, GpControlConfig config, uint32_t now, bool decoder)
{
    memset(f, 0, sizeof(*f));
    current = f;
    f->now = now;
    f->inputs = (GpControlInputs){true, true, true, false, GP_ESTOP_REQUIRED};
    f->inputs_read_ok = true;
    f->discard_ok = true;
    GpControlPort port = {f, enter, leave, clock_ms, notify, read_inputs, discard_rx};
    CHECK(GpControl_Init(&f->control, &config, &port) == GP_OK);
    CHECK(GpCanRx_Init(&f->rx, &f->control, decoder ? decode : NULL) ==
          (decoder ? GP_OK : GP_BAD_ARG));
    CommandMapConfig mapping = {1000, 1000, 10000, 10000, 40};
    MotorDriverConfig driver = {write_motor, off, f, 600, 20000, 5, 2, 3, 10};
    f->context = MOTOR;
    CHECK(GpMotor_Init(&f->motor, &f->control, &mapping, &driver) == GP_OK);
    f->context = TEST;
    CHECK(f->motor.state == GP_DISARMED && !nonzero(f->physical));
    CHECK(f->off_count == 1 && f->writes == 0);
}

static void init(Fixture *f)
{
    init_with(f, GpControl_DefaultConfig(), 0, true);
}

static void init_codec(Fixture *f)
{
    init(f);
    f->production_codec = true;
    CHECK(GpCanRx_Init(&f->rx, &f->control, GP_DecodeCan) == GP_OK);
    CHECK(f->rx.decode == GP_DecodeCan);
}

static GpCanFrame frame_for(Fixture *f, uint16_t sequence, uint16_t throttle,
                            int16_t steering, uint8_t flags)
{
    if (f->production_codec) {
        GpCanFrame frame;
        TargetSample sample = {sequence, throttle, steering, flags};
        CHECK(GP_EncodeCan(&sample, &frame) == GP_OK);
        return frame;
    }
    CHECK(f->sample_count < sizeof(f->samples) / sizeof(f->samples[0]));
    unsigned index = f->sample_count++;
    f->samples[index] = (TargetSample){sequence, throttle, steering, flags};
    f->decode_results[index] = GP_OK;
    GpCanFrame frame = {GP_CAN_COMMAND_ID, 8, {0}};
    frame.data[0] = (uint8_t)index;
    frame.data[1] = (uint8_t)(index >> 8);
    return frame;
}

static GpResult capture(Fixture *f, const GpCanFrame *frame, uint32_t now,
                        bool extended, bool remote)
{
    enum Context old = f->context;
    f->context = ISR;
    f->now = now;
    GpResult result = GpControl_CaptureIsr(&f->control, frame, extended, remote);
    f->context = old;
    return result;
}

static size_t receive(Fixture *f)
{
    enum Context old = f->context;
    f->context = RX;
    size_t count = GpCanRx_Run(&f->rx);
    f->context = old;
    return count;
}

static void queue(Fixture *f, uint16_t seq, uint16_t throttle, int16_t steering,
                  uint8_t flags, uint32_t now)
{
    GpCanFrame frame = frame_for(f, seq, throttle, steering, flags);
    CHECK(capture(f, &frame, now, false, false) == GP_OK);
}

static void send(Fixture *f, uint16_t seq, uint16_t throttle, int16_t steering,
                 uint8_t flags, uint32_t now)
{
    queue(f, seq, throttle, steering, flags, now);
    CHECK(receive(f) != 0);
}

static void arm_at(Fixture *f, uint16_t seq, uint32_t now)
{
    send(f, seq, 0, 0, GP_FLAG_VALID | GP_FLAG_DEADMAN, now);
    CHECK(motor_run(f, now) == GP_OK);
    CHECK(f->motor.state == GP_WAIT_FOR_ARM_EDGE);
    send(f, (uint16_t)(seq + 1u), 500, 0, FLAGS, now + 1u);
    f->allow_motion = true;
    CHECK(motor_run(f, now + 1u) == GP_OK);
    CHECK(f->motor.state == GP_ARMED);
    CHECK(motor_run(f, now + 11u) == GP_OK);
    CHECK(f->physical.throttle_duty_pm == 100);
}

static void arm(Fixture *f)
{
    arm_at(f, 100, 0);
}

static void fault(Fixture *f, uint32_t reason)
{
    enum Context old = f->context;
    f->context = ISR;
    GpControl_LatchFault(&f->control, reason, true);
    f->context = old;
    f->allow_motion = false;
}

static GpResult recover(Fixture *f)
{
    f->context = MOTOR;
    GpResult result = GpMotor_Recover(&f->motor);
    f->context = TEST;
    return result;
}

static void stopped(Fixture *f)
{
    CHECK(!nonzero(f->physical) && !nonzero(f->motor.applied));
    CHECK(!nonzero(f->motor.driver.previous) && !nonzero(f->motor.map.previous));
    CHECK(f->motor.shutdown_executed);
    CHECK(f->motor.state != GP_ARMED);
}

static void startup(void)
{
    Fixture f;
    init(&f);
    send(&f, 1, 1000, 0, FLAGS, 0);
    CHECK(motor_run(&f, 0) == GP_OK);
    CHECK(f.motor.state == GP_WAIT_FOR_NEUTRAL && f.writes == 0);
    CHECK(motor_run(&f, 100) == GP_OK);
    send(&f, 2, 500, 0, GP_FLAG_VALID, 100);
    CHECK(motor_run(&f, 100) == GP_OK);
    send(&f, 3, 500, 0, FLAGS, 101);
    CHECK(motor_run(&f, 101) == GP_OK);
    CHECK(f.motor.state != GP_ARMED);
    arm_at(&f, 4, 102);
}

static void throttle(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 900, 0, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 200);
    for (uint32_t t = 31; t <= 91; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 600);
}

static void steering(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 500, 500, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_PENDING);
    CHECK(!nonzero(f.physical));
    CHECK(motor_run(&f, 31) == GP_OK);
    CHECK(f.physical.steering_rate_pm == 100);
    send(&f, 103, 500, -500, FLAGS, 40);
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(f.physical.steering_rate_pm == 0);
    CHECK(motor_run(&f, 51) == GP_OK);
    CHECK(f.physical.steering_rate_pm == -100);
    CHECK(motor_run(&f, 61) == GP_OK);
    CHECK(f.physical.steering_rate_pm == -200);
}

static void ordinary_neutral(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    unsigned off_before = f.off_count;
    send(&f, 102, 0, 0, FLAGS, 12);
    send(&f, 103, 500, 0, FLAGS, 12);
    CHECK(motor_run(&f, 12) == GP_OK);
    CHECK(f.off_count == off_before + 1);
    CHECK(!nonzero(f.physical));
    CHECK(f.motor.state == GP_ARMED && f.motor.latched_faults == 0);
    CHECK(motor_run(&f, 22) == GP_OK);
    CHECK(nonzero(f.physical));
}

static void stop_throttle(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 0, 0, 0, 12);
    send(&f, 103, 900, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(f.control.urgent_reasons & GP_REASON_STOP);
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults == 0);
    send(&f, 104, 900, 0, FLAGS, 13);
    CHECK(motor_run(&f, 13) == GP_OK);
    stopped(&f);
}

static void stop_arm_burst(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 0, 0, 0, 12);
    queue(&f, 103, 0, 0, GP_FLAG_VALID, 12);
    queue(&f, 104, 800, 0, FLAGS, 12);
    CHECK(receive(&f) == 3);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.control.raw_count == 0 && f.control.event_count == 0);
    CHECK(f.motor.state == GP_WAIT_FOR_NEUTRAL);
    CHECK(motor_run(&f, 22) == GP_OK);
    stopped(&f);
    arm_at(&f, 105, 23);
}

static void stop_then_recovery_queued(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 0, 0, 0, 12);
    /* Recovery is still raw and not decoded when shutdown occurs. */
    queue(&f, 103, 0, 0, GP_FLAG_VALID, 12);
    queue(&f, 104, 800, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    CHECK(receive(&f) == 0);
    stopped(&f);
    arm_at(&f, 105, 13);
}

static void raw_full_stop(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    for (unsigned i = 0; i < f.control.config.raw_capacity; ++i) {
        queue(&f, (uint16_t)(102 + i), 900, 0, FLAGS, 12);
    }
    GpCanFrame stop = frame_for(&f, 120, 0, 0, 0);
    CHECK(capture(&f, &stop, 12, false, false) == GP_NO_SPACE);
    CHECK(f.control.urgent_reasons & GP_FAULT_RAW_OVERFLOW);
    CHECK(f.isr_notifications != 0);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_RAW_OVERFLOW);
}

static void event_full_stop(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    for (unsigned i = 0; i < f.control.config.event_capacity; ++i) {
        send(&f, (uint16_t)(102 + i), 900, 0, FLAGS, 12);
    }
    send(&f, 120, 0, 0, 0, 12);
    CHECK(f.control.urgent_reasons & GP_FAULT_EVENT_OVERFLOW);
    CHECK(f.control.urgent_reasons & GP_REASON_STOP);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
}

static void duplicate(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 101, 0, 0, 0, 20); /* replayed STOP is not fresh */
    CHECK(f.control.event_count == 0 && f.control.urgent_reasons == 0);
    CHECK(motor_run(&f, 21) == GP_OK);
    CHECK(f.motor.latest_received_ms == 1);
    for (uint32_t t = 31; t <= 191; t += 10) {
        send(&f, 101, 500, 0, FLAGS, t);
        CHECK(motor_run(&f, t) == GP_OK);
        CHECK(f.motor.latest_received_ms == 1);
    }
    f.allow_motion = false;
    CHECK(motor_run(&f, 201) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_TIMEOUT);
    stopped(&f);
}

static void sequence_wrap(void)
{
    Fixture f;
    init(&f);
    arm_at(&f, 65534, 0);
    send(&f, 0, 500, 0, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_OK);
    CHECK(f.motor.latest.sequence == 0);
    send(&f, 32767, 500, 0, FLAGS, 30);
    CHECK(motor_run(&f, 31) == GP_OK);
    CHECK(f.motor.latest.sequence == 32767);
    send(&f, 65535, 0, 0, 0, 40); /* delta 32768 rejected */
    CHECK(f.control.urgent_reasons == 0 && f.control.event_count == 0);
}

static void out_of_order(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 100, 0, 0, 0, 20);
    send(&f, 99, 1000, 0, FLAGS, 20);
    CHECK(f.control.event_count == 0 && f.control.urgent_reasons == 0);
    CHECK(motor_run(&f, 21) == GP_OK);
    CHECK(f.motor.latest.sequence == 101 && f.motor.latest_received_ms == 1);
}

static void stale_raw(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 900, 0, FLAGS, 12);
    f.now = 53;
    CHECK(receive(&f) == 1);
    CHECK(f.control.urgent_reasons & GP_FAULT_QUEUE_AGE);
    f.allow_motion = false;
    CHECK(motor_run(&f, 53) == GP_INVALID);
    stopped(&f);
}

static void stale_event(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 900, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 53) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_QUEUE_AGE);
    stopped(&f);
}

static void timestamp_capture(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 500, 0, FLAGS, 12);
    f.now = 22;
    CHECK(receive(&f) == 1);
    CHECK(motor_run(&f, 32) == GP_OK);
    CHECK(f.motor.latest_received_ms == 12);
    CHECK(f.motor.latest_received_ms != f.now);
}

static void timeout(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    for (uint32_t t = 21; t <= 191; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    /* A new frame queued after the lapse cannot conceal the timeout. */
    send(&f, 102, 900, 0, FLAGS, 201);
    f.allow_motion = false;
    CHECK(motor_run(&f, 201) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_TIMEOUT);
}

static void deadman(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 500, 0, GP_FLAG_VALID | GP_FLAG_ARM, 12);
    send(&f, 103, 500, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults == 0);
    CHECK(f.motor.last_shutdown_reasons & GP_REASON_PERMISSION);
}

static void arm_edge_order(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 0, 0, GP_FLAG_VALID, 12);
    send(&f, 103, 500, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.state == GP_WAIT_FOR_NEUTRAL);
    arm_at(&f, 104, 13);
}

static void estop(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.inputs.estop_ok = false;
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_ESTOP);
    CHECK(recover(&f) == GP_INVALID);
    f.inputs.estop_ok = true;
    CHECK(motor_run(&f, 13) == GP_INVALID);
    CHECK(recover(&f) == GP_OK);
    stopped(&f);
    arm_at(&f, 1, 14);
}

static void bus_off(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    unsigned count = f.off_count;
    fault(&f, GP_FAULT_BUS_OFF | GP_FAULT_CAN_OVERFLOW);
    CHECK(f.off_count == count); /* ISR never writes outputs */
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK((f.motor.latched_faults & (GP_FAULT_BUS_OFF | GP_FAULT_CAN_OVERFLOW)) ==
          (GP_FAULT_BUS_OFF | GP_FAULT_CAN_OVERFLOW));
    f.inputs.can_bus_ok = false;
    CHECK(recover(&f) == GP_INVALID);
    f.inputs.can_bus_ok = true;
    CHECK(recover(&f) == GP_OK);
}

static void sender_reset(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 0, 0, 0, GP_FLAG_VALID, 20);
    CHECK(f.control.event_count == 0);
    for (uint32_t t = 21; t <= 191; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    f.allow_motion = false;
    CHECK(motor_run(&f, 201) == GP_INVALID);
    CHECK(recover(&f) == GP_OK);
    send(&f, 1, 900, 0, FLAGS, 202);
    CHECK(motor_run(&f, 202) == GP_OK);
    stopped(&f);
    arm_at(&f, 2, 203);
}

static void driver_error(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.driver_result = GP_IO_ERROR;
    CHECK(motor_run(&f, 21) == GP_IO_ERROR);
    f.allow_motion = false;
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_OUTPUT);
    f.driver_result = GP_OK;
    CHECK(motor_run(&f, 31) == GP_INVALID);
    stopped(&f);
}

static void pending(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.driver_result = GP_PENDING;
    CHECK(motor_run(&f, 21) == GP_PENDING);
    CHECK(!nonzero(f.physical) && f.motor.latched_faults == 0);
    CHECK(motor_run(&f, 31) == GP_PENDING);
    f.driver_result = GP_OK;
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(nonzero(f.physical));
}

static void fault_pending(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    send(&f, 102, 500, 500, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_PENDING);
    fault(&f, GP_FAULT_CAN_OVERFLOW);
    CHECK(motor_run(&f, 22) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.driver.last_steering_direction == 0);
    CHECK(motor_run(&f, 31) == GP_INVALID);
    stopped(&f);
}

static void urgent_notification(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    unsigned before = f.notifications[GP_WAKE_MOTOR];
    fault(&f, GP_FAULT_RAW_OVERFLOW);
    CHECK(f.notifications[GP_WAKE_MOTOR] == before + 1);
    CHECK(f.control.urgent_reasons & GP_FAULT_RAW_OVERFLOW);
    unsigned writes = f.writes;
    CHECK(motor_run(&f, 12) == GP_INVALID); /* earlier than 21ms deadline */
    CHECK(f.writes == writes);
    stopped(&f);
}

static void backlog(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 900, 0, FLAGS, 20);
    f.allow_motion = false;
    CHECK(motor_run(&f, 21) == GP_PENDING);
    CHECK(f.motor.latched_faults == 0 && f.motor.rx_waiting);
    CHECK(!nonzero(f.physical) && !nonzero(f.motor.applied));
    CHECK(f.control.raw_count == 1); /* preserve the undecoded packet */
    CHECK(receive(&f) == 1);
    f.allow_motion = true;
    CHECK(motor_run(&f, 31) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 100); /* restart slew from zero */
    CHECK(!f.motor.rx_waiting && f.motor.rx_wait_count == 1);
}

static void event_budget(void)
{
    Fixture f;
    GpControlConfig config = GpControl_DefaultConfig();
    config.motor_event_budget = 1;
    init_with(&f, config, 0, true);
    arm(&f);
    send(&f, 102, 500, 0, FLAGS, 20);
    send(&f, 103, 600, 0, FLAGS, 20);
    f.allow_motion = false;
    CHECK(motor_run(&f, 21) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_BACKLOG);
}

static void tick_wrap(void)
{
    Fixture f;
    uint32_t start = UINT32_MAX - 5u;
    init_with(&f, GpControl_DefaultConfig(), start, true);
    arm_at(&f, 65534, start);
    CHECK(f.now == 5);
    send(&f, 0, 500, 0, FLAGS, 6);
    CHECK(motor_run(&f, 15) == GP_OK);
    CHECK(f.motor.latest_received_ms == 6);
    for (uint32_t t = 25; t <= 195; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    f.allow_motion = false;
    CHECK(motor_run(&f, 206) == GP_INVALID);
    stopped(&f);
}

static void repeated_shutdown(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    for (unsigned i = 0; i < 5; ++i) {
        fault(&f, GP_FAULT_ESTOP);
        CHECK(motor_run(&f, 12 + i) == GP_INVALID);
        stopped(&f);
        CHECK(f.motor.shutdown_count == i + 1);
    }
    CHECK(recover(&f) == GP_OK);
    stopped(&f);
    CHECK(f.motor.fault_history & GP_FAULT_ESTOP);
}

static void recover_barrier(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    fault(&f, GP_FAULT_BUS_OFF);
    CHECK(motor_run(&f, 12) == GP_INVALID);
    queue(&f, 102, 0, 0, GP_FLAG_VALID, 13);
    queue(&f, 103, 900, 0, FLAGS, 13);
    CHECK(recover(&f) == GP_OK);
    CHECK(receive(&f) == 0);
    CHECK(motor_run(&f, 14) == GP_OK);
    stopped(&f);
    arm_at(&f, 0, 15);
}

static void shutdown_during_decode(Fixture *f)
{
    fault(f, GP_FAULT_ESTOP);
    CHECK(motor_run(f, f->now) == GP_INVALID);
}

static void inflight_generation(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 0, 0, GP_FLAG_VALID, 12);
    f.decode_hook = shutdown_during_decode;
    CHECK(receive(&f) == 1);
    CHECK(f.control.event_count == 0);
    stopped(&f);
    CHECK(recover(&f) == GP_OK);
    send(&f, 103, 900, 0, FLAGS, 13);
    CHECK(motor_run(&f, 13) == GP_OK);
    stopped(&f);
    arm_at(&f, 104, 14);
}

static void motor_preempts_decode(Fixture *f)
{
    f->allow_motion = false;
    CHECK(motor_run(f, f->now) == GP_PENDING);
    CHECK(f->motor.latched_faults == 0 && !nonzero(f->physical));
}

static void inflight_backlog(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 900, 0, FLAGS, 21);
    f.decode_hook = motor_preempts_decode;
    CHECK(receive(&f) == 1);
    CHECK(!nonzero(f.physical));
    CHECK(f.control.event_count == 1);
    f.allow_motion = true;
    CHECK(motor_run(&f, 31) == GP_OK);
    CHECK(nonzero(f.physical) && f.motor.latched_faults == 0);
}

static void fault_between_checks(Fixture *f)
{
    fault(f, GP_FAULT_BUS_OFF);
}

static void final_commit_fault(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    unsigned writes = f.writes;
    f.unlocked_hook = fault_between_checks;
    CHECK(motor_run(&f, 21) == GP_INVALID);
    CHECK(f.writes == writes);
    stopped(&f);
}

static void frame_between_checks(Fixture *f)
{
    queue(f, 102, 0, 0, 0, f->now);
    f->allow_motion = false;
}

static void final_commit_frame(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.unlocked_hook = frame_between_checks;
    CHECK(motor_run(&f, 21) == GP_PENDING);
    CHECK(f.motor.latched_faults == 0 && !nonzero(f.physical));
    CHECK(receive(&f) == 1); /* STOP survives the race and is honored */
    CHECK(motor_run(&f, 22) == GP_INVALID);
    CHECK(f.motor.last_shutdown_reasons == GP_REASON_STOP);
    stopped(&f);
}

static void scheduled_notifications(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    queue(&f, 102, 0, 0, 0, 12);
    queue(&f, 103, 0, 0, GP_FLAG_VALID, 12);
    queue(&f, 104, 900, 0, FLAGS, 12);
    f.allow_motion = false;
    f.auto_motor_notify = true;
    CHECK(receive(&f) == 1); /* STOP notification preempts RX, flushes rest */
    stopped(&f);
    CHECK(f.control.event_count == 0 && f.control.raw_count == 0);
}

static void malformed(void)
{
    for (unsigned kind = 0; kind < 9; ++kind) {
        Fixture f;
        init(&f);
        arm(&f);
        GpCanFrame frame = frame_for(&f, 102, 500, 0, FLAGS);
        switch (kind) {
        case 0: frame.id = 0x106; break;
        case 1: frame.dlc = 9; break;
        case 2: frame.dlc = 7; break;
        case 3: f.decode_results[f.sample_count - 1] = GP_INVALID; break;
        case 4: f.samples[f.sample_count - 1].flags = 0x80; break;
        case 5: f.samples[f.sample_count - 1].throttle_pm = 1001; break;
        case 6: f.samples[f.sample_count - 1].flags = 0; break; /* invalid non-neutral */
        default: break;
        }
        CHECK(capture(&f, &frame, 20, kind == 7, kind == 8) == GP_OK);
        CHECK(receive(&f) == 1);
        CHECK(f.control.event_count == 0 && f.control.urgent_reasons == 0);
        CHECK(motor_run(&f, 21) == GP_OK);
        CHECK(f.motor.latest_received_ms == 1);
    }
}

static void missing_dependencies(void)
{
    Fixture f;
    init_with(&f, GpControl_DefaultConfig(), 0, false);
    CHECK(motor_run(&f, 0) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_DEPENDENCY);
    stopped(&f);
    CHECK(recover(&f) == GP_INVALID);
    init(&f);
    f.inputs.outputs_approved = false;
    CHECK(motor_run(&f, 0) == GP_INVALID);
    stopped(&f);
    init(&f);
    f.inputs_read_ok = false;
    CHECK(motor_run(&f, 0) == GP_INVALID);
    stopped(&f);
}

static void hardware_backlog(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.inputs.hardware_rx_pending = true;
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_PENDING);
    CHECK(f.motor.latched_faults == 0 && !nonzero(f.physical));
    unsigned offs = f.off_count;
    CHECK(motor_run(&f, 52) == GP_PENDING); /* bounded, exact 40ms */
    CHECK(f.off_count == offs); /* do not restart the driver's stop each poll */
    CHECK(motor_run(&f, 53) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_BACKLOG);
    stopped(&f);
}

static void rx_wait_bound(void)
{
    Fixture f;
    uint32_t start = UINT32_MAX - 20u;
    init_with(&f, GpControl_DefaultConfig(), start, true);
    arm_at(&f, 100, start);
    queue(&f, 102, 900, 0, FLAGS, start + 12u);
    f.allow_motion = false;
    CHECK(motor_run(&f, start + 12u) == GP_PENDING);
    CHECK(motor_run(&f, start + 52u) == GP_PENDING);
    CHECK(f.motor.rx_wait_count == 1 && f.control.raw_count == 1);
    CHECK(motor_run(&f, start + 53u) == GP_INVALID);
    CHECK(f.motor.latched_faults == GP_FAULT_BACKLOG);
    stopped(&f);
    CHECK(f.control.raw_count == 0);
}

static void decoded_between_checks(Fixture *f)
{
    send(f, 102, 900, 0, FLAGS, f->now);
}

static void hardware_between_checks(Fixture *f)
{
    f->inputs.hardware_rx_pending = true;
}

static void rx_commit_races(void)
{
    for (unsigned hardware = 0; hardware < 2; ++hardware) {
        Fixture f;
        init(&f);
        arm(&f);
        unsigned writes = f.writes;
        f.unlocked_hook = hardware ? hardware_between_checks : decoded_between_checks;
        CHECK(motor_run(&f, 21) == GP_PENDING);
        CHECK(f.writes == writes && !nonzero(f.physical));
        CHECK(f.motor.rx_waiting && f.motor.latched_faults == 0);
        if (hardware) {
            f.inputs.hardware_rx_pending = false; /* ISR drains FIFO */
            send(&f, 102, 900, 0, FLAGS, 22);
        }
        CHECK(motor_run(&f, 22) == GP_OK);
        CHECK(!nonzero(f.physical)); /* no invented slew credit */
        CHECK(motor_run(&f, 31) == GP_OK);
        CHECK(f.physical.throttle_duty_pm == 100);
        CHECK(f.motor.latest.throttle_pm == 900 && !f.motor.rx_waiting);
    }
}

static void rx_continuous(void)
{
    Fixture f;
    GpControlConfig config = GpControl_DefaultConfig();
    config.direct_commands = true;
    init_with(&f, config, 0, true);
    f.production_codec = true;
    CHECK(GpCanRx_Init(&f.rx, &f.control, GP_DecodeCan) == GP_OK);
    f.allow_motion = true;
    /* 20 seconds of actual wire frames, deliberately motor-first every 20ms.
     * No recovery, dropped packet, sequence reset or operator permission. */
    for (uint16_t seq = 1; seq <= 1000; ++seq) {
        uint32_t now = (uint32_t)seq * 20u;
        queue(&f, seq, 500, 0, GP_FLAG_VALID, now);
        CHECK(motor_run(&f, now) == GP_PENDING);
        CHECK(!nonzero(f.physical));
        CHECK(receive(&f) == 1);
        CHECK(motor_run(&f, now + 1) == GP_OK);
        CHECK(motor_run(&f, now + 11) == GP_OK);
        CHECK(nonzero(f.physical));
        CHECK(f.motor.latched_faults == 0 && !f.motor.rx_waiting);
    }
    CHECK(f.motor.rx_wait_count == 1000 && f.control.decoded == 1000);
    CHECK(f.control.raw_dropped == 0 && f.control.barred == 0);
}

static void deadline(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.allow_motion = false;
    CHECK(motor_run(&f, 52) == GP_INVALID); /* dt=41 exceeds mapper bound */
    CHECK(f.motor.latched_faults & GP_FAULT_TIMING);
    stopped(&f);
}

static void bounded_rx(void)
{
    Fixture f;
    GpControlConfig config = GpControl_DefaultConfig();
    config.rx_budget = 2;
    init_with(&f, config, 0, true);
    queue(&f, 1, 0, 0, GP_FLAG_VALID, 0);
    queue(&f, 2, 0, 0, GP_FLAG_VALID, 0);
    queue(&f, 3, 0, 0, GP_FLAG_VALID, 0);
    unsigned before = f.notifications[GP_WAKE_CAN_RX];
    CHECK(receive(&f) == 2);
    CHECK(f.control.raw_count == 1);
    CHECK(f.notifications[GP_WAKE_CAN_RX] == before + 1);
    CHECK(receive(&f) == 1);
    CHECK(motor_run(&f, 0) == GP_OK);
}

static void config_arguments(void)
{
    Fixture f;
    init(&f);
    GpControl other;
    GpControlConfig config = GpControl_DefaultConfig();
    GpControlPort port = f.control.port;
    CHECK(GpControl_Init(NULL, &config, &port) == GP_BAD_ARG);
    CHECK(GpControl_Init(&other, NULL, &port) == GP_BAD_ARG);
    port.enter = NULL;
    CHECK(GpControl_Init(&other, &config, &port) == GP_BAD_ARG);
    port = f.control.port;
    config.raw_capacity = 0;
    CHECK(GpControl_Init(&other, &config, &port) == GP_INVALID);
    config = GpControl_DefaultConfig();
    config.motor_period_ms = config.command_timeout_ms;
    CHECK(GpControl_Init(&other, &config, &port) == GP_INVALID);
    CHECK(GpMotor_Run(NULL) == GP_BAD_ARG);
    CHECK(GpMotor_Recover(NULL) == GP_BAD_ARG);
    CHECK(GpCanRx_Run(NULL) == 0);
    CHECK(GpControl_CaptureIsr(NULL, NULL, false, false) == GP_BAD_ARG);
    GpControl_LatchFault(NULL, GP_FAULT_ESTOP, true);
    CHECK(recover(&f) == GP_INVALID);
    printf("storage bytes: channels=%lu rx=%lu motor=%lu total=%lu\n",
           (unsigned long)sizeof(GpControl), (unsigned long)sizeof(GpCanRxTask),
           (unsigned long)sizeof(GpMotorTask),
           (unsigned long)(sizeof(GpControl) + sizeof(GpCanRxTask) + sizeof(GpMotorTask)));
}

static void hardware_barrier(void)
{
    Fixture f;
    init(&f);
    arm(&f);
    f.fifo_buffered = 3;
    send(&f, 102, 0, 0, 0, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    CHECK(f.fifo_buffered == 0);
    stopped(&f);
    arm_at(&f, 103, 13);
    f.discard_ok = false;
    send(&f, 105, 0, 0, 0, 25);
    f.allow_motion = false;
    CHECK(motor_run(&f, 25) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_DEPENDENCY);
    CHECK(recover(&f) == GP_INVALID);
    stopped(&f);
    f.discard_ok = true;
    CHECK(recover(&f) == GP_OK);
    stopped(&f);
}

static void pending_reversal(void)
{
    for (unsigned inject_fault = 0; inject_fault < 2; ++inject_fault) {
        Fixture f;
        init(&f);
        /* Synthetic test timing, configured before movement. */
        f.motor.driver.config.reversal_deadtime_ms = 30;
        arm(&f);
        send(&f, 102, 500, 100, FLAGS, 20);
        CHECK(motor_run(&f, 21) == GP_PENDING);
        CHECK(motor_run(&f, 31) == GP_PENDING);
        CHECK(motor_run(&f, 41) == GP_PENDING);
        CHECK(motor_run(&f, 51) == GP_OK);
        CHECK(f.physical.steering_rate_pm == 100);
        send(&f, 103, 500, -100, FLAGS, 52);
        CHECK(motor_run(&f, 61) == GP_OK);
        CHECK(f.physical.steering_rate_pm == 0);
        CHECK(motor_run(&f, 71) == GP_PENDING);
        CHECK(!nonzero(f.physical) && f.motor.latched_faults == 0);
        if (inject_fault) {
            fault(&f, GP_FAULT_ESTOP);
            CHECK(motor_run(&f, 72) == GP_INVALID);
            stopped(&f);
            CHECK(motor_run(&f, 91) == GP_INVALID);
            stopped(&f);
        } else {
            CHECK(motor_run(&f, 81) == GP_PENDING);
            CHECK(motor_run(&f, 91) == GP_OK);
            CHECK(f.physical.steering_rate_pm == -100);
        }
    }
}

static void moving_both(Fixture *f, bool codec)
{
    if (codec) init_codec(f); else init(f);
    arm(f);
    send(f, 102, 500, 500, FLAGS, 20);
    CHECK(motor_run(f, 21) == GP_PENDING);
    CHECK(motor_run(f, 31) == GP_OK);
    /* Both slew histories restart at zero after the pending interval. */
    CHECK(f->physical.throttle_duty_pm == 100 && f->physical.steering_rate_pm == 100);
}

static void neutral_steering(void)
{
    Fixture f;
    moving_both(&f, false);
    unsigned off_before = f.off_count;
    uint32_t generation = f.control.generation;
    send(&f, 103, 500, 0, FLAGS, 31); /* dt=0 must still stop this axis */
    CHECK(motor_run(&f, 31) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 100 && f.physical.steering_rate_pm == 0);
    CHECK(f.off_count == off_before && f.motor.state == GP_ARMED);
    CHECK(f.control.generation == generation && f.motor.shutdown_count == 0);
    CHECK(f.motor.last_update_ms == 31);
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 200 && f.physical.steering_rate_pm == 0);
}

static void neutral_throttle(void)
{
    Fixture f;
    moving_both(&f, false);
    unsigned off_before = f.off_count;
    send(&f, 103, 0, 500, FLAGS, 32);
    CHECK(motor_run(&f, 32) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 0 && f.physical.steering_rate_pm == 100);
    CHECK(f.off_count == off_before && f.motor.state == GP_ARMED);
    CHECK(f.motor.last_update_ms == 31);
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 0 && f.physical.steering_rate_pm == 200);
}

static void neutral_both(void)
{
    Fixture f;
    moving_both(&f, false);
    unsigned off_before = f.off_count;
    uint32_t generation = f.control.generation;
    send(&f, 103, 0, 0, FLAGS, 32);
    CHECK(motor_run(&f, 32) == GP_OK);
    CHECK(!nonzero(f.physical) && !nonzero(f.motor.map.previous));
    CHECK(f.off_count == off_before + 1 && f.motor.state == GP_ARMED);
    CHECK(f.motor.latched_faults == 0 && f.control.generation == generation);
    CHECK(f.motor.shutdown_count == 0);
}

static void neutral_due(void)
{
    for (unsigned axis = 0; axis < 2; ++axis) {
        Fixture f;
        moving_both(&f, false);
        unsigned off_before = f.off_count;
        send(&f, 103, axis == 0 ? 0 : 500, axis == 1 ? 0 : 500, FLAGS, 41);
        CHECK(motor_run(&f, 41) == GP_OK);
        CHECK(f.physical.throttle_duty_pm == (axis == 0 ? 0 : 200));
        CHECK(f.physical.steering_rate_pm == (axis == 1 ? 0 : 200));
        CHECK(f.off_count == off_before && f.motor.state == GP_ARMED);
        CHECK(f.motor.last_update_ms == 41);
    }
}

static void neutral_coalesced(void)
{
    for (unsigned axis = 0; axis < 2; ++axis) {
        Fixture f;
        moving_both(&f, false);
        unsigned off_before = f.off_count;
        send(&f, 103, axis == 0 ? 0 : 500, axis == 1 ? 0 : 500, FLAGS, 32);
        send(&f, 104, 500, 500, FLAGS, 32);
        CHECK(motor_run(&f, 32) == GP_OK);
        CHECK(f.physical.throttle_duty_pm == (axis == 0 ? 0 : 100));
        CHECK(f.physical.steering_rate_pm == (axis == 1 ? 0 : 100));
        CHECK(f.off_count == off_before && f.motor.state == GP_ARMED);
        CHECK(f.motor.latest.throttle_pm == 500 && f.motor.latest.steering_pm == 500);
        CHECK(motor_run(&f, 41) == (axis == 1 ? GP_PENDING : GP_OK));
        /* Steering restart still obeys Boda's both-off deadtime contract. */
        CHECK(motor_run(&f, 51) == GP_OK);
        CHECK(f.physical.throttle_duty_pm > 0);
        CHECK(f.physical.steering_rate_pm > 0);
    }
    Fixture f;
    moving_both(&f, false);
    send(&f, 103, 0, 500, FLAGS, 32);
    send(&f, 104, 500, 0, FLAGS, 32);
    send(&f, 105, 500, 500, FLAGS, 32);
    CHECK(motor_run(&f, 32) == GP_OK);
    CHECK(!nonzero(f.physical)); /* neither ordered zero can be hidden */
    CHECK(f.motor.state == GP_ARMED);
}

static void neutral_fractional(void)
{
    Fixture f;
    moving_both(&f, false);
    f.motor.map.config.throttle_slew_pm_s = 1; /* synthetic slow axis */
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(f.motor.map.throttle_remainder == 10);
    send(&f, 103, 500, 0, FLAGS, 41);
    CHECK(motor_run(&f, 41) == GP_OK);
    CHECK(f.motor.map.throttle_remainder == 10 && f.motor.map.steering_remainder == 0);
    CHECK(f.physical.throttle_duty_pm == 100 && f.physical.steering_rate_pm == 0);
    CHECK(motor_run(&f, 51) == GP_OK);
    CHECK(f.motor.map.throttle_remainder == 20);
}

static void neutral_pending(void)
{
    Fixture f;
    init(&f);
    f.motor.driver.config.reversal_deadtime_ms = 30;
    arm(&f);
    send(&f, 102, 500, 500, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_PENDING);
    send(&f, 103, 500, 0, FLAGS, 22);
    CHECK(motor_run(&f, 22) == GP_OK); /* cancel steering; next period ramps throttle */
    CHECK(f.physical.throttle_duty_pm == 0 && f.physical.steering_rate_pm == 0);
    send(&f, 104, 500, -500, FLAGS, 23);
    CHECK(motor_run(&f, 31) == GP_PENDING); /* driver contract still stops BOTH */
    CHECK(!nonzero(f.physical));
    send(&f, 105, 0, -500, FLAGS, 32);
    CHECK(motor_run(&f, 32) == GP_PENDING);
    CHECK(!nonzero(f.physical) && f.motor.state == GP_ARMED);
    fault(&f, GP_FAULT_ESTOP);
    CHECK(motor_run(&f, 33) == GP_INVALID);
    stopped(&f);
}

static void neutral_driver_error(void)
{
    Fixture f;
    moving_both(&f, false);
    send(&f, 103, 500, 0, FLAGS, 32);
    f.driver_result = GP_IO_ERROR;
    f.allow_motion = true; /* attempted request is permitted; adapter fails */
    CHECK(motor_run(&f, 32) == GP_IO_ERROR);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_OUTPUT);
}

static void neutral_final_fault(void)
{
    Fixture f;
    moving_both(&f, false);
    send(&f, 103, 500, 0, FLAGS, 32);
    unsigned writes = f.writes;
    f.unlocked_hook = fault_between_checks;
    CHECK(motor_run(&f, 32) == GP_INVALID);
    CHECK(f.writes == writes);
    stopped(&f);
}

static void codec_path(void)
{
    Fixture f;
    moving_both(&f, true);
    CHECK(f.decode_calls == 0); /* test double was never used */
    CHECK(f.motor.latest.sequence == 102 && f.motor.latest_received_ms == 20);
    send(&f, 103, 0, 0, 0, 32); /* well-formed VALID-low STOP */
    CHECK(f.control.urgent_reasons & GP_REASON_STOP);
    f.allow_motion = false;
    CHECK(motor_run(&f, 32) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.last_shutdown_reasons & GP_REASON_STOP);
    CHECK(f.motor.latched_faults == 0);
}

static void codec_rejected_freshness(void)
{
    Fixture f;
    init_codec(&f);
    arm(&f);
    for (uint32_t t = 21; t <= 191; t += 10) {
        for (unsigned kind = 0; kind < 9; ++kind) {
            GpCanFrame frame = frame_for(&f, 102, 500, 0, FLAGS);
            switch (kind) {
            case 0: frame.data[7] ^= 1; break;
            case 1: frame.id = 0x106; break;
            case 2: frame.dlc = 7; break;
            case 3: frame.dlc = 9; break;
            case 4: frame = frame_for(&f, 101, 500, 0, FLAGS); break; /* duplicate */
            case 5: frame = frame_for(&f, 100, 500, 0, FLAGS); break; /* older */
            case 8: /* valid CRC, invalid nonneutral VALID-low payload */
                frame = (GpCanFrame){0x105, 8, {1, 0, 0x58, 2, 0x38, 0xff, 7, 0x62}};
                frame.data[6] = 0;
                /* CRC-8 over the above payload with flags zero. */
                frame.data[7] = 0x77;
                {
                    TargetSample ignored;
                    CHECK(GP_DecodeCan(&frame, &ignored) == GP_INVALID);
                }
                break;
            default: break;
            }
            CHECK(capture(&f, &frame, t, kind == 6, kind == 7) == GP_OK);
            CHECK(receive(&f) == 1);
            CHECK(f.control.event_count == 0 && f.control.urgent_reasons == 0);
        }
        CHECK(motor_run(&f, t) == GP_OK);
        CHECK(f.motor.latest_received_ms == 1 && f.rx.last_sequence == 101);
    }
    f.allow_motion = false;
    CHECK(motor_run(&f, 201) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_TIMEOUT);
    stopped(&f);
}

static void codec_sequence(void)
{
    Fixture f;
    init_codec(&f);
    arm_at(&f, 65534, 0);
    send(&f, 0, 500, 0, FLAGS, 20);
    CHECK(motor_run(&f, 21) == GP_OK);
    CHECK(f.motor.latest.sequence == 0 && f.motor.latest_received_ms == 20);
    send(&f, 0, 0, 0, 0, 22); /* replayed STOP cannot renew freshness */
    send(&f, 65535, 500, 0, FLAGS, 22);
    CHECK(f.control.event_count == 0 && f.control.urgent_reasons == 0);
    CHECK(f.motor.latest_received_ms == 20);
    send(&f, 1, 500, 0, FLAGS, 23); /* rejected frames did not consume sequence */
    CHECK(motor_run(&f, 23) == GP_OK);
    CHECK(f.motor.latest.sequence == 1);
}

static void codec_stop_barrier(void)
{
    Fixture f;
    init_codec(&f);
    arm(&f);
    uint32_t generation = f.control.generation;
    for (unsigned i = 0; i < f.control.config.event_capacity; ++i) {
        send(&f, (uint16_t)(102 + i), 500, 0, FLAGS, 12);
    }
    send(&f, 120, 0, 0, 0, 12);
    CHECK((f.control.urgent_reasons & (GP_REASON_STOP | GP_FAULT_EVENT_OVERFLOW)) ==
          (GP_REASON_STOP | GP_FAULT_EVENT_OVERFLOW));
    queue(&f, 121, 0, 0, GP_FLAG_VALID, 12);
    queue(&f, 122, 900, 0, FLAGS, 12);
    f.allow_motion = false;
    CHECK(motor_run(&f, 12) == GP_INVALID);
    stopped(&f);
    CHECK(f.control.generation != generation && receive(&f) == 0);
    CHECK(recover(&f) == GP_OK);
    send(&f, 123, 900, 0, FLAGS, 13);
    CHECK(motor_run(&f, 13) == GP_OK);
    stopped(&f);
    arm_at(&f, 124, 14);
}

static void codec_fault_recovery(void)
{
    Fixture f;
    init_codec(&f);
    arm(&f);
    fault(&f, GP_FAULT_BUS_OFF);
    CHECK(motor_run(&f, 12) == GP_INVALID);
    queue(&f, 102, 0, 0, GP_FLAG_VALID, 13);
    queue(&f, 103, 500, 0, FLAGS, 13);
    CHECK(recover(&f) == GP_OK);
    CHECK(receive(&f) == 0);
    stopped(&f);
    send(&f, 104, 500, 0, FLAGS, 14);
    CHECK(motor_run(&f, 14) == GP_OK);
    stopped(&f);
    arm_at(&f, 105, 15);
}

static void codec_neutral(void)
{
    Fixture f;
    moving_both(&f, true);
    unsigned off_before = f.off_count;
    send(&f, 103, 0, 500, FLAGS, 32);
    CHECK(motor_run(&f, 32) == GP_OK);
    CHECK(f.physical.throttle_duty_pm == 0 && f.physical.steering_rate_pm == 100);
    send(&f, 104, 0, 0, FLAGS, 33);
    CHECK(motor_run(&f, 33) == GP_OK);
    CHECK(!nonzero(f.physical) && f.motor.state == GP_ARMED);
    CHECK(f.off_count == off_before + 1 && f.motor.shutdown_count == 0);
}

static void inhibited_supervisor(void)
{
    Fixture f;
    init_codec(&f);
    f.context = MOTOR;
    CHECK(GpMotor_InitInhibited(&f.motor, &f.control, off, &f) == GP_OK);
    f.context = TEST;
    CHECK(f.motor.state == GP_FAULT_SHUTDOWN && !f.motor.driver.initialized);
    CHECK(f.motor.latched_faults & GP_FAULT_DEPENDENCY);
    /* Even the fixture's healthy inputs cannot turn monitoring into actuation. */
    send(&f, 1, 500, 300, FLAGS, 1);
    CHECK(f.control.decoded == 1 && f.control.barred == 1);
    CHECK(f.control.last_decoded.throttle_pm == 500);
    CHECK(f.control.published == 0 && !f.motor.have_latest);
    CHECK(motor_run(&f, 1) == GP_INVALID);
    f.context = MOTOR;
    CHECK(GpMotor_Recover(&f.motor) == GP_INVALID);
    f.context = TEST;
    CHECK(f.motor.state == GP_FAULT_SHUTDOWN && f.writes == 0);
    CHECK(GpMotor_InitInhibited(NULL, &f.control, off, &f) == GP_BAD_ARG);
}

static void unwired_estop(void)
{
    Fixture f;
    init_codec(&f);
    f.inputs.estop_mode = GP_ESTOP_NOT_CONNECTED;
    f.inputs.estop_ok = false; /* absent, never fabricated as healthy */
    arm(&f);
    CHECK(f.physical.throttle_duty_pm > 0 && !f.inputs.estop_ok);
    fault(&f, GP_FAULT_ESTOP); /* explicit faults still latch in every mode */
    CHECK(motor_run(&f, 12) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_ESTOP);
    CHECK(recover(&f) == GP_OK);
    send(&f, 102, 500, 0, FLAGS, 13);
    CHECK(motor_run(&f, 13) == GP_OK);
    stopped(&f); /* recovery alone/ARM-high cannot authorize motion */
    arm_at(&f, 103, 14);
    f.inputs.outputs_approved = false;
    f.allow_motion = false;
    CHECK(motor_run(&f, 26) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_DEPENDENCY);
    CHECK(recover(&f) == GP_INVALID);
}
static void unknown_estop_mode(void)
{
    Fixture f;
    init(&f);
    f.inputs.estop_mode = (GpEstopMode)99;
    CHECK(motor_run(&f, 0) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_DEPENDENCY);
    stopped(&f);
}
static void diagnostic_counters(void)
{
    Fixture f;
    init_codec(&f);
    arm(&f);
    uint32_t before = f.control.published;
    uint32_t timestamp = f.motor.latest_received_ms;
    send(&f, 101, 400, 0, FLAGS, 2); /* duplicate: decoded but never published */
    CHECK(f.control.stale == 1 && f.control.published == before);
    GpCanFrame bad = frame_for(&f, 102, 400, 0, FLAGS);
    bad.data[7] ^= 1;
    CHECK(capture(&f, &bad, 3, false, false) == GP_OK);
    CHECK(memcmp(f.control.last_rx_data, bad.data, sizeof(bad.data)) == 0);
    CHECK(f.control.last_rx_ms == 3);
    CHECK(receive(&f) == 1);
    CHECK(f.control.rejected == 1 && f.control.published == before);
    CHECK(f.motor.latest_received_ms == timestamp);
    CHECK(f.control.captured == f.control.decoded + f.control.rejected);
    CHECK(f.control.last_rx_id == GP_CAN_COMMAND_ID && f.control.last_rx_dlc == 8);
    GpCanFrame unrelated = {0x321, 2, {0}};
    CHECK(capture(&f, &unrelated, 4, false, false) == GP_OK);
    CHECK(receive(&f) == 1);
    CHECK(f.control.last_rx_id == 0x321 && f.control.last_rx_dlc == 2);
    CHECK(f.motor.latest_received_ms == timestamp); /* RX metadata is not freshness */
    CHECK(memcmp(f.control.last_rx_data, unrelated.data, sizeof(unrelated.data)) == 0);
    CHECK(f.control.last_rx_ms == 4 && !f.control.last_rx_extended && !f.control.last_rx_remote);
    fault(&f, GP_FAULT_ESTOP);
    CHECK(motor_run(&f, 5) == GP_INVALID);
    CHECK(f.control.last_rx_ms == 4); /* shutdown preserves raw diagnostic evidence */
}

static void direct_commands(void)
{
    Fixture f;
    GpControlConfig cfg = GpControl_DefaultConfig();
    cfg.direct_commands = true;
    init_with(&f, cfg, 0, true);
    f.production_codec = true;
    CHECK(GpCanRx_Init(&f.rx, &f.control, GP_DecodeCan) == GP_OK);
    f.allow_motion = true;
    send(&f, 1, 500, 0, GP_FLAG_VALID, 1);
    CHECK(motor_run(&f, 1) == GP_OK && f.motor.state == GP_ARMED);
    CHECK(motor_run(&f, 11) == GP_OK && f.physical.throttle_duty_pm == 100);
    CHECK(f.motor.latest.flags == GP_FLAG_VALID); /* never fabricate ARM/DEADMAN */
    /* Switching either operator flag has no effect in direct mode. */
    send(&f, 2, 500, 0, FLAGS, 21);
    CHECK(motor_run(&f, 21) == GP_OK);
    send(&f, 3, 500, 0, GP_FLAG_VALID, 31);
    CHECK(motor_run(&f, 31) == GP_OK && f.physical.throttle_duty_pm == 300);
    /* STOP still wins, and resumption needs a new valid command (no handshake). */
    send(&f, 4, 0, 0, 0, 32);
    CHECK(motor_run(&f, 32) == GP_INVALID);
    stopped(&f);
    send(&f, 5, 500, 0, GP_FLAG_VALID, 33);
    CHECK(motor_run(&f, 33) == GP_OK);
    CHECK(motor_run(&f, 43) == GP_OK && f.physical.throttle_duty_pm == 100);
    /* Duplicate and malformed commands cannot keep an old output alive. */
    for (uint32_t t = 53; t <= 93; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    send(&f, 5, 500, 0, GP_FLAG_VALID, 100);
    CHECK(f.control.stale == 1 && f.motor.latest_received_ms == 33);
    GpCanFrame bad = frame_for(&f, 6, 500, 0, GP_FLAG_VALID);
    bad.data[7] ^= 1;
    CHECK(capture(&f, &bad, 101, false, false) == GP_OK);
    CHECK(receive(&f) == 1 && f.control.rejected == 1);
    for (uint32_t t = 103; t < 233; t += 10) CHECK(motor_run(&f, t) == GP_OK);
    f.allow_motion = false;
    CHECK(motor_run(&f, 233) == GP_INVALID);
    CHECK(f.motor.latched_faults & GP_FAULT_TIMEOUT);
    stopped(&f);
    send(&f, 7, 500, 0, GP_FLAG_VALID, 234);
    CHECK(motor_run(&f, 234) == GP_OK); /* fresh direct command resumes */
    CHECK(f.motor.latched_faults == 0 && !nonzero(f.physical));
    f.allow_motion = true;
    send(&f, 8, 500, 0, GP_FLAG_VALID, 235);
    CHECK(motor_run(&f, 235) == GP_OK);
    CHECK(motor_run(&f, 245) == GP_OK && f.physical.throttle_duty_pm > 0);
    fault(&f, GP_FAULT_ESTOP);
    CHECK(motor_run(&f, 246) == GP_INVALID);
    stopped(&f);
    CHECK(f.motor.latched_faults & GP_FAULT_ESTOP);
}

static void direct_stream_resume(void)
{
    const uint32_t reasons[] = {GP_FAULT_TIMEOUT, GP_FAULT_QUEUE_AGE,
                               GP_FAULT_BACKLOG, GP_FAULT_TIMING};
    for (unsigned i = 0; i < sizeof(reasons) / sizeof(reasons[0]); ++i) {
        Fixture f;
        GpControlConfig cfg = GpControl_DefaultConfig();
        cfg.direct_commands = true;
        init_with(&f, cfg, 0, true);
        f.production_codec = true;
        CHECK(GpCanRx_Init(&f.rx, &f.control, GP_DecodeCan) == GP_OK);
        send(&f, 1, 0, 700, GP_FLAG_VALID, 1);
        f.allow_motion = true;
        for (uint32_t t = 1; t <= 51; t += 10) {
            GpResult r = motor_run(&f, t);
            CHECK(r == GP_OK || r == GP_PENDING);
        }
        CHECK(f.physical.steering_rate_pm > 0);
        /* Old queued commands cannot restart a faulted generation. */
        send(&f, 2, 0, 800, GP_FLAG_VALID, 52);
        fault(&f, reasons[i]);
        CHECK(motor_run(&f, 52) == GP_INVALID);
        stopped(&f);
        CHECK(f.motor.latched_faults == reasons[i]);
        CHECK(f.control.event_count == 0 && f.control.accepting);
        CHECK(motor_run(&f, 53) == GP_INVALID); /* no command, no restart */
        GpCanFrame bad = frame_for(&f, 3, 0, -700, GP_FLAG_VALID);
        bad.data[7] ^= 1;
        CHECK(capture(&f, &bad, 54, false, false) == GP_OK);
        CHECK(receive(&f) == 1 && f.control.rejected == 1);
        CHECK(motor_run(&f, 54) == GP_INVALID);
        /* An event that expired during a stalled motor task is never applied. */
        send(&f, 4, 0, -700, GP_FLAG_VALID, 55);
        CHECK(motor_run(&f, 96) == GP_INVALID);
        CHECK(f.control.event_count == 0);
        send(&f, 5, 0, -700, GP_FLAG_VALID, 97);
        f.allow_motion = true;
        CHECK(motor_run(&f, 97) == GP_OK);
        CHECK(f.motor.latest.steering_pm == -700 && f.motor.latched_faults == 0);
        for (uint32_t t = 107; t <= 147; t += 10) {
            GpResult r = motor_run(&f, t);
            CHECK(r == GP_OK || r == GP_PENDING);
        }
        CHECK(f.physical.steering_rate_pm < 0 && f.physical.throttle_duty_pm == 0);
        CHECK(f.motor.fault_history & reasons[i]);
    }
    /* Neither pure hardware faults nor mixed transient/hardware faults resume. */
    const uint32_t hard[] = {GP_FAULT_OUTPUT, GP_FAULT_BUS_OFF, GP_FAULT_ESTOP,
        GP_FAULT_DEPENDENCY, GP_FAULT_RAW_OVERFLOW, GP_FAULT_EVENT_OVERFLOW,
        GP_FAULT_CAN_OVERFLOW, GP_FAULT_TIMING | GP_FAULT_OUTPUT};
    for (unsigned i = 0; i < sizeof(hard) / sizeof(hard[0]); ++i) {
        Fixture f;
        GpControlConfig cfg = GpControl_DefaultConfig();
        cfg.direct_commands = true;
        init_with(&f, cfg, 0, true);
        fault(&f, hard[i]);
        CHECK(motor_run(&f, 1) == GP_INVALID);
        send(&f, 1, 0, 700, GP_FLAG_VALID, 2);
        CHECK(motor_run(&f, 2) == GP_INVALID && f.motor.latched_faults == hard[i]);
        CHECK(!f.control.accepting);
        stopped(&f);
    }
}

static void ready_task_order(void)
{
    Fixture f;
    /* Either ready-task ordering must tolerate one ordinary fresh frame. */
    init_codec(&f);
    queue(&f, 1, 0, 0, GP_FLAG_VALID, 10);
    CHECK(motor_run(&f, 10) == GP_PENDING);
    CHECK(f.motor.latched_faults == 0 && !nonzero(f.physical));
    CHECK(receive(&f) == 1);
    CHECK(motor_run(&f, 10) == GP_OK);
    CHECK(f.motor.state == GP_WAIT_FOR_ARM_EDGE);

    /* Model highest-ready dispatch using the exact firmware priority settings.
     * This is a deterministic ordering test, not a simulated FreeRTOS kernel. */
    init_codec(&f);
    for (uint16_t seq = 1; seq <= 4; ++seq) {
        uint32_t now = 10u * seq;
        uint8_t flags = seq == 1 ? GP_FLAG_VALID : seq == 4 ? 0 : FLAGS;
        queue(&f, seq, seq == 3 ? 500 : 0, 0, flags, now);
        if (GP_MOTOR_TASK_PRIORITY >= GP_CAN_TASK_PRIORITY)
            (void)motor_run(&f, now);
        CHECK(receive(&f) == 1);
        f.allow_motion = seq == 2 || seq == 3;
        GpResult result = motor_run(&f, now);
        CHECK(f.motor.latched_faults == 0);
        if (seq == 1) CHECK(f.motor.state == GP_WAIT_FOR_ARM_EDGE);
        if (seq == 2) CHECK(f.motor.state == GP_ARMED);
        if (seq == 3) CHECK(f.physical.throttle_duty_pm > 0);
        if (seq == 4) {
            CHECK(result == GP_INVALID && f.motor.last_shutdown_reasons == GP_REASON_STOP);
            stopped(&f);
        } else CHECK(result == GP_OK);
    }
}

int main(int argc, char **argv)
{
    const struct { const char *name; void (*run)(void); } tests[] = {
#define TEST_CASE(name) {#name, name}
        TEST_CASE(direct_commands), TEST_CASE(direct_stream_resume), TEST_CASE(ready_task_order), TEST_CASE(startup), TEST_CASE(throttle), TEST_CASE(steering),
        TEST_CASE(ordinary_neutral), TEST_CASE(stop_throttle), TEST_CASE(stop_arm_burst),
        TEST_CASE(stop_then_recovery_queued), TEST_CASE(raw_full_stop), TEST_CASE(event_full_stop),
        TEST_CASE(duplicate), TEST_CASE(sequence_wrap), TEST_CASE(out_of_order),
        TEST_CASE(stale_raw), TEST_CASE(stale_event), TEST_CASE(timestamp_capture),
        TEST_CASE(timeout), TEST_CASE(deadman), TEST_CASE(arm_edge_order),
        TEST_CASE(estop), TEST_CASE(bus_off), TEST_CASE(sender_reset),
        TEST_CASE(driver_error), TEST_CASE(pending), TEST_CASE(fault_pending),
        TEST_CASE(urgent_notification), TEST_CASE(backlog), TEST_CASE(event_budget),
        TEST_CASE(tick_wrap), TEST_CASE(repeated_shutdown), TEST_CASE(recover_barrier),
        TEST_CASE(inflight_generation), TEST_CASE(inflight_backlog),
        TEST_CASE(final_commit_fault), TEST_CASE(final_commit_frame),
        TEST_CASE(scheduled_notifications), TEST_CASE(malformed),
        TEST_CASE(missing_dependencies), TEST_CASE(hardware_backlog),
        TEST_CASE(rx_wait_bound), TEST_CASE(rx_commit_races), TEST_CASE(rx_continuous),
        TEST_CASE(deadline), TEST_CASE(bounded_rx), TEST_CASE(config_arguments),
        TEST_CASE(hardware_barrier), TEST_CASE(pending_reversal),
        TEST_CASE(neutral_steering), TEST_CASE(neutral_throttle), TEST_CASE(neutral_both),
        TEST_CASE(neutral_due),
        TEST_CASE(neutral_coalesced), TEST_CASE(neutral_fractional), TEST_CASE(neutral_pending),
        TEST_CASE(neutral_driver_error), TEST_CASE(neutral_final_fault),
        TEST_CASE(inhibited_supervisor), TEST_CASE(diagnostic_counters),
        TEST_CASE(unwired_estop), TEST_CASE(unknown_estop_mode),
        TEST_CASE(codec_path), TEST_CASE(codec_rejected_freshness), TEST_CASE(codec_sequence),
        TEST_CASE(codec_stop_barrier), TEST_CASE(codec_fault_recovery), TEST_CASE(codec_neutral)
#undef TEST_CASE
    };
    CHECK(argc == 2);
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (strcmp(argv[1], tests[i].name) == 0) {
            tests[i].run();
            printf("PASS %s\n", tests[i].name);
            return 0;
        }
    }
    return 2;
}
