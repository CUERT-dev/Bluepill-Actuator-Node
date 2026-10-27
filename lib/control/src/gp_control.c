#include "gp_control.h"
#include "command_map.h"

#include <string.h>

#define GP_ALLOWED_FLAGS (GP_FLAG_VALID | GP_FLAG_ARM | GP_FLAG_DEADMAN)
#define GP_DISARM_REASONS (GP_REASON_STOP | GP_REASON_PERMISSION)
#define GP_HALF_TIME 0x80000000u
#define GP_NEUTRAL_THROTTLE 1u
#define GP_NEUTRAL_STEERING 2u
#define GP_DIRECT_RETRY_FAULTS (GP_FAULT_TIMEOUT | GP_FAULT_QUEUE_AGE | \
                               GP_FAULT_BACKLOG | GP_FAULT_TIMING)

static bool direct_retry(const GpMotorTask *task)
{
    return task->control->config.direct_commands && task->latched_faults != 0 &&
           (task->latched_faults & ~(uint32_t)GP_DIRECT_RETRY_FAULTS) == 0;
}

static bool neutral(const TargetSample *command)
{
    return command->throttle_pm == 0 && command->steering_pm == 0;
}

static bool age_over(uint32_t now, uint32_t then, uint32_t budget)
{
    /* Future timestamps also fail closed. Service clock within half its range. */
    return (uint32_t)(now - then) > budget;
}

GpControlConfig GpControl_DefaultConfig(void)
{
    GpControlConfig config = {10, 200, 40, 8, 8, 8, 8, false};
    return config;
}

GpResult GpControl_Init(GpControl *control, const GpControlConfig *config,
                        const GpControlPort *port)
{
    if (control == NULL || config == NULL || port == NULL ||
        port->enter == NULL || port->leave == NULL || port->now_ms == NULL ||
        port->notify == NULL || port->read_inputs == NULL || port->discard_rx == NULL) {
        return GP_BAD_ARG;
    }
    if (config->motor_period_ms == 0 ||
        config->motor_period_ms >= config->command_timeout_ms ||
        config->command_timeout_ms >= GP_HALF_TIME ||
        config->stage_budget_ms == 0 ||
        config->stage_budget_ms >= config->command_timeout_ms ||
        config->raw_capacity == 0 || config->raw_capacity > GP_CONTROL_RAW_MAX ||
        config->event_capacity == 0 || config->event_capacity > GP_CONTROL_EVENT_MAX ||
        config->rx_budget == 0 || config->rx_budget > GP_CONTROL_RAW_MAX ||
        config->motor_event_budget == 0 ||
        config->motor_event_budget > GP_CONTROL_EVENT_MAX) {
        return GP_INVALID;
    }
    GpControlPort port_copy = *port;
    GpControlConfig config_copy = *config;
    memset(control, 0, sizeof(*control));
    control->port = port_copy;
    control->config = config_copy;
    control->initialized = true;
    return GP_OK;
}

void GpControl_LatchFault(GpControl *control, uint32_t reasons, bool from_isr)
{
    if (control == NULL || !control->initialized || reasons == 0) {
        return;
    }
    uint32_t saved = control->port.enter(control->port.user, from_isr);
    control->urgent_reasons |= reasons;
    control->port.leave(control->port.user, from_isr, saved);
    control->port.notify(control->port.user, GP_WAKE_MOTOR, from_isr);
}

GpResult GpControl_CaptureIsr(GpControl *control, const GpCanFrame *frame,
                             bool extended, bool remote)
{
    if (control == NULL || !control->initialized || frame == NULL) {
        return GP_BAD_ARG;
    }
    uint32_t received = control->port.now_ms(control->port.user);
    uint32_t saved = control->port.enter(control->port.user, true);
    bool full = control->raw_count == control->config.raw_capacity;
    ++control->captured;
    control->last_rx_id = frame->id;
    control->last_rx_dlc = frame->dlc;
    memcpy(control->last_rx_data, frame->data, sizeof(control->last_rx_data));
    control->last_rx_ms = received;
    control->last_rx_extended = extended;
    control->last_rx_remote = remote;
    if (full) {
        ++control->raw_dropped;
        control->urgent_reasons |= GP_FAULT_RAW_OVERFLOW;
    } else {
        uint16_t index = (uint16_t)((control->raw_head + control->raw_count) %
                                    control->config.raw_capacity);
        control->raw[index] = (GpRawCan){*frame, received, control->generation,
                                         extended, remote};
        ++control->raw_count;
    }
    control->port.leave(control->port.user, true, saved);
    control->port.notify(control->port.user, full ? GP_WAKE_MOTOR : GP_WAKE_CAN_RX, true);
    return full ? GP_NO_SPACE : GP_OK;
}

GpResult GpCanRx_Init(GpCanRxTask *task, GpControl *control, GpCanDecodeFn decode)
{
    if (task == NULL || control == NULL || !control->initialized) {
        return GP_BAD_ARG;
    }
    memset(task, 0, sizeof(*task));
    task->control = control;
    task->decode = decode;
    uint32_t saved = control->port.enter(control->port.user, false);
    control->decoder_ready = decode != NULL;
    control->port.leave(control->port.user, false, saved);
    return decode == NULL ? GP_BAD_ARG : GP_OK;
}

size_t GpCanRx_Run(GpCanRxTask *task)
{
    if (task == NULL || task->control == NULL || task->decode == NULL) {
        return 0;
    }
    GpControl *control = task->control;
    size_t processed = 0;
    bool published = false;
    for (; processed < control->config.rx_budget; ++processed) {
        uint32_t saved = control->port.enter(control->port.user, false);
        if (control->raw_count == 0) {
            control->port.leave(control->port.user, false, saved);
            break;
        }
        GpRawCan raw = control->raw[control->raw_head];
        control->raw_head = (uint16_t)((control->raw_head + 1u) % control->config.raw_capacity);
        --control->raw_count;
        control->rx_inflight = true;
        control->inflight_generation = raw.generation;
        control->port.leave(control->port.user, false, saved);

        TargetSample command = {0};
        bool format = !raw.extended && !raw.remote &&
                      raw.frame.id == GP_CAN_COMMAND_ID &&
                      raw.frame.dlc > 0 && raw.frame.dlc <= sizeof(raw.frame.data);
        bool decoded = format && task->decode(&raw.frame, &command) == GP_OK;
        bool content = decoded && command.throttle_pm <= 1000 &&
                       command.steering_pm >= -1000 && command.steering_pm <= 1000 &&
                       (command.flags & ~GP_ALLOWED_FLAGS) == 0 &&
                       ((command.flags & GP_FLAG_VALID) != 0 || neutral(&command));

        uint32_t now = control->port.now_ms(control->port.user);
        bool notify = false;
        saved = control->port.enter(control->port.user, false);
        control->rx_inflight = false;
        if (content) {
            ++control->decoded;
            control->last_decoded = command;
            control->last_decoded_ms = raw.received_ms;
        } else {
            ++control->rejected;
        }
        if (raw.generation != control->generation || !control->accepting)
            ++control->barred;
        /* An old in-flight decode cannot publish into a new recovery epoch. */
        if (raw.generation == control->generation && control->accepting) {
            if (task->generation != raw.generation) {
                task->generation = raw.generation;
                task->seen = false;
            }
            if (age_over(now, raw.received_ms, control->config.stage_budget_ms)) {
                control->urgent_reasons |= GP_FAULT_QUEUE_AGE;
                notify = true;
            } else if (content) {
                uint16_t delta = (uint16_t)(command.sequence - task->last_sequence);
                bool fresh = !task->seen || (delta != 0 && delta < 0x8000u);
                if (fresh) {
                    /* STOP reaches the latch even if the event queue is full. */
                    bool stop = (command.flags & GP_FLAG_VALID) == 0;
                    if (stop) {
                        control->urgent_reasons |= GP_REASON_STOP;
                    }
                    if (control->event_count == control->config.event_capacity) {
                        control->urgent_reasons |= GP_FAULT_EVENT_OVERFLOW;
                    } else {
                        uint16_t index = (uint16_t)((control->event_head + control->event_count) %
                                                    control->config.event_capacity);
                        control->events[index] = (GpCommandEvent){command, raw.received_ms,
                                                                  now, raw.generation};
                        ++control->event_count;
                        ++control->published;
                        task->seen = true;
                        task->last_sequence = command.sequence;
                    }
                    /* Every accepted event wakes motor: safety transitions
                     * are processed before considering any target coalescing. */
                    notify = control->urgent_reasons != 0;
                    published = true;
                } else {
                    ++control->stale;
                }
            }
        }
        control->port.leave(control->port.user, false, saved);
        if (notify) {
            control->port.notify(control->port.user, GP_WAKE_MOTOR, false);
        }
    }
    uint32_t saved = control->port.enter(control->port.user, false);
    bool remaining = control->raw_count != 0;
    control->port.leave(control->port.user, false, saved);
    if (remaining) {
        control->port.notify(control->port.user, GP_WAKE_CAN_RX, false);
    }
    if (published) {
        control->port.notify(control->port.user, GP_WAKE_MOTOR, false);
    }
    return processed;
}

/* Caller holds channel lock; only motor context calls these helpers. */
static bool cut_generation(GpControl *control)
{
    bool discarded = control->port.discard_rx(control->port.user);
    ++control->generation;
    control->raw_head = control->raw_count = 0;
    control->event_head = control->event_count = 0;
    /* Do not clear rx_inflight: decoder might still own a previous generation. */
    return discarded;
}

static void shutdown(GpMotorTask *task, uint32_t reasons, uint32_t now)
{
    GpControl *control = task->control;
    /* The ordering here is the recovery barrier. No candidate survives it. */
    if (task->inhibited_off) task->inhibited_off(task->inhibited_user);
    else MotorDriver_Stop(&task->driver);
    CommandMap_Reset(&task->map);
    if (!cut_generation(control)) reasons |= GP_FAULT_DEPENDENCY;
    reasons |= control->urgent_reasons;
    task->applied = (MotorOutput){0, 0};
    task->latest = (TargetSample){0};
    task->have_latest = false;
    task->output_pending = false;
    task->rx_waiting = false;
    task->last_update_ms = now;
    task->last_shutdown_reasons = reasons;
    task->fault_history |= reasons & ~(uint32_t)GP_DISARM_REASONS;
    task->latched_faults |= reasons & ~(uint32_t)GP_DISARM_REASONS;
    ++task->shutdown_count;
    task->shutdown_executed = true;
    control->urgent_reasons = 0;
    task->state = task->latched_faults ? GP_FAULT_SHUTDOWN : GP_WAIT_FOR_NEUTRAL;
    /* Direct commands need no local acknowledgement after a transient stream
     * stop. The generation barrier still discards everything already buffered;
     * only a newly captured, decoded and fresh command can restart motion. */
    control->accepting = task->latched_faults == 0 || direct_retry(task);
}

static uint32_t input_faults(GpControl *control, bool *rx_pending)
{
    GpControlInputs inputs = {0};
    *rx_pending = false;
    if (!control->port.read_inputs(control->port.user, &inputs)) {
        return GP_FAULT_DEPENDENCY;
    }
    uint32_t faults = 0;
    if (inputs.estop_mode == GP_ESTOP_REQUIRED) {
        if (!inputs.estop_ok) faults |= GP_FAULT_ESTOP;
    } else if (inputs.estop_mode != GP_ESTOP_NOT_CONNECTED) {
        faults |= GP_FAULT_DEPENDENCY;
    }
    if (!inputs.can_bus_ok) faults |= GP_FAULT_BUS_OFF;
    if (!inputs.outputs_approved || !control->decoder_ready) faults |= GP_FAULT_DEPENDENCY;
    *rx_pending = inputs.hardware_rx_pending;
    return faults;
}

static bool raw_backlog(const GpControl *control)
{
    return control->raw_count != 0 ||
           (control->rx_inflight && control->inflight_generation == control->generation);
}

/* A frame racing a motor deadline is ordinary traffic, not an overload.
 * Hold BOTH outputs off without flushing the packet (it could be STOP).
 * Decoder/ISR notifications and periodic motor runs finish the transaction.
 * No slew credit accumulates while waiting; persistent congestion still latches.
 * Caller holds the channel lock. Never notify a worker under that lock. */
static GpResult wait_for_rx(GpMotorTask *task, uint32_t now)
{
    uint32_t throttle_remainder = task->map.throttle_remainder;
    uint32_t steering_remainder = task->map.steering_remainder;
    if (!task->rx_waiting) {
        task->rx_waiting = true;
        task->rx_wait_since_ms = now;
        ++task->rx_wait_count;
        /* A pending driver transaction already holds both axes off. Keep its
         * setup/deadtime progress so repeated CAN arrivals cannot starve it. */
        if (!task->output_pending) MotorDriver_Stop(&task->driver);
    }
    CommandMap_Reset(&task->map);
    /* Retain only previously earned sub-unit credit, including startup at
     * very low slew. Clearing it at every packet could prevent any first step. */
    task->map.throttle_remainder = throttle_remainder;
    task->map.steering_remainder = steering_remainder;
    task->applied = (MotorOutput){0, 0};
    task->last_update_ms = now;
    return GP_PENDING;
}

static bool command_permission(const GpMotorTask *task)
{
    uint8_t required = task->control->config.direct_commands ? GP_FLAG_VALID : GP_ALLOWED_FLAGS;
    return task->state == GP_ARMED && task->have_latest && task->latched_faults == 0 &&
           (task->latest.flags & required) == required;
}

GpResult GpMotor_InitInhibited(GpMotorTask *task, GpControl *control,
                              void (*off)(void *), void *user)
{
    if (!task || !control || !control->initialized || !off) return GP_BAD_ARG;
    memset(task, 0, sizeof(*task));
    task->control = control;
    task->inhibited_off = off;
    task->inhibited_user = user;
    task->initialized = true;
    uint32_t saved = control->port.enter(control->port.user, false);
    shutdown(task, GP_FAULT_DEPENDENCY, control->port.now_ms(control->port.user));
    control->port.leave(control->port.user, false, saved);
    return GP_OK;
}

GpResult GpMotor_Init(GpMotorTask *task, GpControl *control,
                      const CommandMapConfig *map_config, const MotorDriverConfig *driver_config)
{
    if (task == NULL || control == NULL || !control->initialized ||
        map_config == NULL || driver_config == NULL) {
        return GP_BAD_ARG;
    }
    memset(task, 0, sizeof(*task));
    task->control = control;
    uint32_t saved = control->port.enter(control->port.user, false);
    GpResult result = MotorDriver_Init(&task->driver, driver_config);
    if (result == GP_OK) {
        result = CommandMap_Init(&task->map, map_config);
    }
    if (result == GP_OK &&
        (map_config->max_dt_ms < control->config.motor_period_ms ||
         map_config->max_dt_ms >= GP_HALF_TIME)) {
        result = GP_INVALID;
    }
    if (result == GP_OK) {
        if (!cut_generation(control)) {
            control->urgent_reasons |= GP_FAULT_DEPENDENCY;
        }
        control->accepting = true;
        task->last_update_ms = control->port.now_ms(control->port.user);
        task->state = GP_DISARMED;
        task->shutdown_executed = true; /* driver Init synchronously called off */
        task->initialized = true;
    } else {
        MotorDriver_Stop(&task->driver);
        control->accepting = false;
    }
    control->port.leave(control->port.user, false, saved);
    return result;
}

/* Every event is inspected in order, even if later targets supersede it. */
static uint32_t accept_event(GpMotorTask *task, const GpCommandEvent *event,
                             uint8_t *neutral_axes)
{
    const TargetSample *command = &event->command;
    bool arm = (command->flags & GP_FLAG_ARM) != 0;
    bool deadman = (command->flags & GP_FLAG_DEADMAN) != 0;
    if ((command->flags & GP_FLAG_VALID) == 0) {
        return GP_REASON_STOP;
    }
    /* Direct mode changes operator authorization only. Fault/freshness/STOP
     * checks still surround event acceptance and the final output commit. */
    if (task->control->config.direct_commands) task->state = GP_ARMED;
    if (task->state == GP_ARMED) {
        if (!task->control->config.direct_commands && (!arm || !deadman)) {
            return GP_REASON_PERMISSION;
        }
        /* Preserve ordered zeros independently, even if later targets in this
         * batch are nonzero. A neutral command is not a safety shutdown. */
        if (neutral(command) || (task->have_latest &&
            task->latest.throttle_pm != 0 && command->throttle_pm == 0)) {
            *neutral_axes |= GP_NEUTRAL_THROTTLE;
        }
        if (neutral(command) || (task->have_latest &&
            task->latest.steering_pm != 0 && command->steering_pm == 0)) {
            *neutral_axes |= GP_NEUTRAL_STEERING;
        }
    } else if (task->state == GP_WAIT_FOR_ARM_EDGE) {
        if (arm && task->have_latest && (task->latest.flags & GP_FLAG_ARM) == 0) {
            if (!deadman) return GP_REASON_PERMISSION;
            task->state = GP_ARMED;
        } else if (!neutral(command) || arm) {
            task->state = GP_WAIT_FOR_NEUTRAL;
        }
    } else if (task->state == GP_WAIT_FOR_NEUTRAL || task->state == GP_DISARMED) {
        task->state = neutral(command) && !arm ? GP_WAIT_FOR_ARM_EDGE : GP_WAIT_FOR_NEUTRAL;
    }
    task->latest = *command;
    task->latest_received_ms = event->received_ms;
    task->have_latest = true;
    return 0;
}

GpResult GpMotor_Run(GpMotorTask *task)
{
    if (task == NULL || !task->initialized) return GP_BAD_ARG;
    GpControl *control = task->control;
    uint32_t saved = control->port.enter(control->port.user, false);
    uint32_t now = control->port.now_ms(control->port.user);
    bool rx_pending;
    uint32_t reasons = input_faults(control, &rx_pending);
    if (task->rx_waiting && age_over(now, task->rx_wait_since_ms,
                                    control->config.stage_budget_ms)) {
        reasons |= GP_FAULT_BACKLOG;
    }
    if (task->inhibited_off) reasons |= GP_FAULT_DEPENDENCY;
    reasons |= control->urgent_reasons;
    if (task->have_latest &&
        (uint32_t)(now - task->latest_received_ms) >= control->config.command_timeout_ms) {
        reasons |= GP_FAULT_TIMEOUT;
    }
    if (reasons != 0) {
        shutdown(task, reasons, now);
        control->port.leave(control->port.user, false, saved);
        return GP_INVALID;
    }
    if (task->latched_faults != 0) {
        bool fresh_restart = false;
        if (direct_retry(task) && control->event_count != 0) {
            const GpCommandEvent *event = &control->events[control->event_head];
            fresh_restart = event->generation == control->generation &&
                (event->command.flags & GP_FLAG_VALID) != 0 &&
                !age_over(now, event->published_ms, control->config.stage_budget_ms) &&
                (uint32_t)(now - event->received_ms) < control->config.command_timeout_ms;
        }
        if (!fresh_restart) {
            if (direct_retry(task) && control->event_count != 0)
                shutdown(task, GP_FAULT_QUEUE_AGE, now);
            control->port.leave(control->port.user, false, saved);
            return GP_INVALID;
        }
        task->latched_faults = 0;
        task->state = GP_DISARMED;
        /* shutdown already reset driver/map and held both outputs inactive. */
    }
    if (task->state == GP_DISARMED) task->state = GP_WAIT_FOR_NEUTRAL;
    bool was_armed = task->state == GP_ARMED;
    uint8_t neutral_axes = 0;
    uint16_t processed = 0;
    while (control->event_count != 0 && processed < control->config.motor_event_budget) {
        GpCommandEvent event = control->events[control->event_head];
        control->event_head = (uint16_t)((control->event_head + 1u) % control->config.event_capacity);
        --control->event_count;
        ++processed;
        if (event.generation != control->generation) continue;
        if (age_over(now, event.published_ms, control->config.stage_budget_ms) ||
            (uint32_t)(now - event.received_ms) >= control->config.command_timeout_ms) {
            reasons |= GP_FAULT_QUEUE_AGE;
            break;
        }
        reasons |= accept_event(task, &event, &neutral_axes);
        if (reasons != 0) break;
    }
    /* Exhausting the configured event budget is an actual overload. */
    if (reasons == 0 && control->event_count != 0) {
        reasons |= GP_FAULT_BACKLOG;
    }
    if (reasons != 0) {
        shutdown(task, reasons, now);
        control->port.leave(control->port.user, false, saved);
        return GP_INVALID;
    }
    if (rx_pending || raw_backlog(control)) {
        GpResult waiting = wait_for_rx(task, now);
        control->port.leave(control->port.user, false, saved);
        control->port.notify(control->port.user, GP_WAKE_CAN_RX, false);
        return waiting;
    }
    if (task->state != GP_ARMED || !task->have_latest) {
        task->rx_waiting = false;
        task->last_update_ms = now;
        control->port.leave(control->port.user, false, saved);
        return GP_OK;
    }
    if (!was_armed) task->last_update_ms = now;
    uint32_t dt = now - task->last_update_ms;
    bool period_due = dt >= control->config.motor_period_ms;
    if (!period_due && neutral_axes == 0) {
        control->port.leave(control->port.user, false, saved);
        return GP_OK; /* no new output due */
    }
    if (!period_due && task->output_pending && !(neutral_axes & GP_NEUTRAL_STEERING)) {
        /* Throttle is already off during steering setup. With cleared slew
         * history, an early throttle neutral must not become an all-zero write
         * that cancels the still-requested steering transaction. */
        task->map.throttle_remainder = 0;
        control->port.leave(control->port.user, false, saved);
        return GP_PENDING;
    }
    if (dt > task->map.config.max_dt_ms) {
        shutdown(task, GP_FAULT_TIMING, now);
        control->port.leave(control->port.user, false, saved);
        return GP_INVALID;
    }
    uint32_t generation = control->generation;
    control->port.leave(control->port.user, false, saved);

    /* Clear only the neutral axis's slew history. An early neutral notification
     * must not advance the other axis's slew or invent elapsed mapping time. */
    CommandMap prior_map = task->map;
    TargetSample command = task->latest;
    if (neutral_axes & GP_NEUTRAL_THROTTLE) {
        command.throttle_pm = 0;
        task->map.previous.throttle_duty_pm = 0;
        task->map.throttle_remainder = 0;
    }
    if (neutral_axes & GP_NEUTRAL_STEERING) {
        command.steering_pm = 0;
        task->map.previous.steering_rate_pm = 0;
        task->map.steering_remainder = 0;
    }
    /* Expensive arithmetic is outside the shared channel critical section. */
    MotorOutput request = task->map.previous;
    uint32_t throttle_remainder = task->map.throttle_remainder;
    uint32_t steering_remainder = task->map.steering_remainder;
    GpResult result = GP_OK;
    if (period_due) {
        result = control->config.direct_commands
            ? CommandMap_UpdateDirect(&task->map, &command, dt, command_permission(task), &request)
            : CommandMap_Update(&task->map, &command, dt, command_permission(task), &request);
    }

    saved = control->port.enter(control->port.user, false);
    now = control->port.now_ms(control->port.user);
    reasons = input_faults(control, &rx_pending);
    reasons |= control->urgent_reasons;
    if (generation != control->generation ||
        (task->rx_waiting && age_over(now, task->rx_wait_since_ms,
                                     control->config.stage_budget_ms))) {
        reasons |= GP_FAULT_BACKLOG;
    }
    if ((uint32_t)(now - task->latest_received_ms) >= control->config.command_timeout_ms) {
        reasons |= GP_FAULT_TIMEOUT;
    }
    if ((uint32_t)(now - task->last_update_ms) > task->map.config.max_dt_ms) {
        reasons |= GP_FAULT_TIMING;
    }
    if (result != GP_OK) reasons |= GP_FAULT_MAPPING;
    if (!command_permission(task)) reasons |= GP_REASON_PERMISSION;
    if (reasons != 0) {
        shutdown(task, reasons, now);
        result = GP_INVALID;
    } else if (rx_pending || control->event_count != 0 || raw_backlog(control)) {
        task->map = prior_map; /* discard arithmetic for the uncommitted request */
        result = wait_for_rx(task, now);
    } else {
        task->rx_waiting = false;
        /* Final check and output commit are serialized against ISR/event
         * publication. Hardware cutoff remains independent of this lock. */
        /* Driver permission is derived from supervisor faults, not a physical
         * readback. An absent E-stop is explicit in GpControlInputs.estop_mode. */
        MotorSafety safety = {(reasons & GP_FAULT_ESTOP) == 0, command_permission(task)};
        result = MotorDriver_Apply(&task->driver, &request, &safety, now, &task->applied);
        task->output_pending = result == GP_PENDING;
        if (result == GP_PENDING) {
            /* The driver's pending contract holds BOTH outputs at zero.
             * Do not accumulate unapplied slew during deadtime/power readiness:
             * restarting must ramp from the output that was actually applied.
             * Reset mapper history only; retain the driver's cancellable phase. */
            CommandMap_Reset(&task->map);
            /* Retry the same fractional increment until it is applied. Without
             * this, a slew below one unit/period repeatedly cancels readiness
             * with zero requests and can never start steering. */
            task->map.throttle_remainder = command.throttle_pm ? throttle_remainder : 0;
            task->map.steering_remainder = command.steering_pm ? steering_remainder : 0;
        }
        if (period_due) task->last_update_ms = now;
        if (result != GP_OK && result != GP_PENDING) {
            shutdown(task, GP_FAULT_OUTPUT, now);
        }
    }
    control->port.leave(control->port.user, false, saved);
    if (task->rx_waiting)
        control->port.notify(control->port.user, GP_WAKE_CAN_RX, false);
    return result;
}

GpResult GpMotor_Recover(GpMotorTask *task)
{
    if (task == NULL || !task->initialized) return GP_BAD_ARG;
    GpControl *control = task->control;
    uint32_t saved = control->port.enter(control->port.user, false);
    bool rx_pending;
    uint32_t reasons = input_faults(control, &rx_pending);
    if (task->inhibited_off) reasons |= GP_FAULT_DEPENDENCY;
    reasons |= control->urgent_reasons;
    uint32_t now = control->port.now_ms(control->port.user);
    if (reasons != 0) {
        shutdown(task, reasons, now);
        control->port.leave(control->port.user, false, saved);
        return GP_INVALID;
    }
    if (rx_pending) {
        control->port.leave(control->port.user, false, saved);
        return GP_PENDING; /* Recovery must still fence the hardware FIFO. */
    }
    if (task->state != GP_FAULT_SHUTDOWN || !task->shutdown_executed) {
        control->port.leave(control->port.user, false, saved);
        return GP_INVALID;
    }
    task->latched_faults = 0;
    shutdown(task, GP_REASON_PERMISSION, now);
    control->port.leave(control->port.user, false, saved);
    return task->latched_faults == 0 ? GP_OK : GP_INVALID;
}
