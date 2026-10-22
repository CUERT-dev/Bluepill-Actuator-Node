#ifndef GP_CONTROL_H
#define GP_CONTROL_H

#include "gp_types.h"

/* Storage limits; runtime capacities/budgets may be smaller. No allocation. */
#ifndef GP_CONTROL_RAW_MAX
#define GP_CONTROL_RAW_MAX 16u
#endif
#ifndef GP_CONTROL_EVENT_MAX
#define GP_CONTROL_EVENT_MAX 16u
#endif

typedef enum {
    GP_DISARMED, GP_WAIT_FOR_NEUTRAL, GP_WAIT_FOR_ARM_EDGE, GP_ARMED,
    GP_FAULT_SHUTDOWN
} GpSafetyState;

/* STOP and PERMISSION are disarming reasons, not latched emergency faults. */
enum {
    GP_REASON_STOP = 1u << 0,
    GP_REASON_PERMISSION = 1u << 1,
    GP_FAULT_RAW_OVERFLOW = 1u << 2,
    GP_FAULT_EVENT_OVERFLOW = 1u << 3,
    GP_FAULT_CAN_OVERFLOW = 1u << 4,
    GP_FAULT_BUS_OFF = 1u << 5,
    GP_FAULT_ESTOP = 1u << 6,
    GP_FAULT_OUTPUT = 1u << 7,
    GP_FAULT_TIMEOUT = 1u << 8,
    GP_FAULT_QUEUE_AGE = 1u << 9,
    GP_FAULT_BACKLOG = 1u << 10,
    GP_FAULT_MAPPING = 1u << 11,
    GP_FAULT_DEPENDENCY = 1u << 12,
    GP_FAULT_TIMING = 1u << 13
};

typedef enum { GP_WAKE_CAN_RX, GP_WAKE_MOTOR } GpWakeTarget;
typedef enum {
    GP_ESTOP_REQUIRED = 0, /* Default: missing/unhealthy readback fails closed. */
    GP_ESTOP_NOT_CONNECTED /* Board capability, never a healthy readback. */
} GpEstopMode;
typedef struct {
    bool estop_ok;
    bool can_bus_ok;
    bool outputs_approved;
    bool hardware_rx_pending;
    GpEstopMode estop_mode;
} GpControlInputs;

/*
 * Mandatory platform boundary, not a supplied hardware implementation.
 * enter/leave must save/restore interrupt state, serialize all shared access,
 * and supply compiler/CPU memory ordering. All publishing ISRs must be covered.
 * from_isr selects legal RTOS primitives. Never use a blocking mutex here.
 * now_ms is one monotonic uint32 millisecond clock in ALL contexts.
 * notify is nonblocking, sticky/coalescible, and called outside the lock.
 * read_inputs and motor callbacks run in a bounded task critical section:
 * no blocking, busy wait, logging, scheduling or autonomous pending activation.
 * estop_mode explicitly distinguishes required readback from absent wiring.
 * estop_ok is meaningful only with GP_ESTOP_REQUIRED. Absence never establishes
 * electrical safety; outputs_approved is software/configuration permission.
 * Independent power interruption is outside the waveform supervisor.
 * discard_rx runs after outputs are off, before a new generation is accepted.
 * It must discard pre-shutdown frames still in hardware FIFO/port buffers in
 * bounded time; false means recovery stays inhibited. No decoder is called.
 */
typedef struct {
    void *user;
    uint32_t (*enter)(void *user, bool from_isr);
    void (*leave)(void *user, bool from_isr, uint32_t saved);
    uint32_t (*now_ms)(void *user);
    void (*notify)(void *user, GpWakeTarget target, bool from_isr);
    bool (*read_inputs)(void *user, GpControlInputs *inputs);
    bool (*discard_rx)(void *user);
} GpControlPort;

typedef struct {
    uint32_t motor_period_ms, command_timeout_ms, stage_budget_ms;
    uint16_t raw_capacity, event_capacity, rx_budget, motor_event_budget;
} GpControlConfig;

typedef struct {
    GpCanFrame frame;
    uint32_t received_ms, generation;
    bool extended, remote;
} GpRawCan;

typedef struct {
    TargetSample command;
    uint32_t received_ms, published_ms, generation;
} GpCommandEvent;

/* Shared channels. Fields are private to the implementation after Init. */
typedef struct {
    GpControlPort port;
    GpControlConfig config;
    GpRawCan raw[GP_CONTROL_RAW_MAX];
    GpCommandEvent events[GP_CONTROL_EVENT_MAX];
    uint16_t raw_head, raw_count, event_head, event_count;
    uint32_t generation, urgent_reasons, inflight_generation;
    bool rx_inflight, decoder_ready, accepting, initialized;
    /* Diagnostics only; updates use the same channel lock. Never grant
     * freshness or output permission from these counters/last decoded fields. */
    uint32_t captured, raw_dropped, decoded, rejected, stale, published, barred;
    TargetSample last_decoded;
    uint32_t last_decoded_ms;
} GpControl;

/* Production GP_DecodeCan from lib/can is passed directly by the app. GP_OK means
 * structurally well-formed, including a VALID-low neutral stop packet.
 * All other results reject the frame; partially filled output is never used.
 * Exact wire layout/checksum/length validation belongs to the real decoder.
 * There is deliberately no production fallback or weak decoder. */
typedef GpResult (*GpCanDecodeFn)(const GpCanFrame *, TargetSample *);

/* Owned exclusively by the CAN task; never shared with the motor task. */
typedef struct {
    GpControl *control;
    GpCanDecodeFn decode;
    uint32_t generation;
    uint16_t last_sequence;
    bool seen;
} GpCanRxTask;

/* Owned exclusively by the motor task, INCLUDING diagnostics and latest.
 * Do not expose pointers to latest/map/driver to other tasks or ISRs. */
typedef struct {
    GpControl *control;
    CommandMap map;
    MotorDriver driver;
    TargetSample latest;
    MotorOutput applied;
    uint32_t latest_received_ms, last_update_ms;
    uint32_t latched_faults, last_shutdown_reasons, fault_history, shutdown_count;
    GpSafetyState state;
    bool have_latest, shutdown_executed, initialized, output_pending;
    void (*inhibited_off)(void *);
    void *inhibited_user;
} GpMotorTask;

/* Only timing/capacity proposals; NO approved motor or electrical defaults. */
GpControlConfig GpControl_DefaultConfig(void);
GpResult GpControl_Init(GpControl *, const GpControlConfig *, const GpControlPort *);
/* Once per captured frame; timestamp is taken here, NOT by a worker.
 * Preserve IDE/RTR metadata separately because GpCanFrame does not contain it.
 * Hardware ISR must use a fixed drain budget and report FIFO overrun separately. */
GpResult GpControl_CaptureIsr(GpControl *, const GpCanFrame *, bool extended, bool remote);
/* Sticky fault indication independent of queue space. No driver calls. */
void GpControl_LatchFault(GpControl *, uint32_t reasons, bool from_isr);

/* Initialize before enabling CAN interrupts. NULL decode leaves dependency
 * unavailable. Run processes at most rx_budget frames and re-notifies if busy. */
GpResult GpCanRx_Init(GpCanRxTask *, GpControl *, GpCanDecodeFn);
size_t GpCanRx_Run(GpCanRxTask *);

/* These three functions may ONLY execute in the motor task.
 * Init calls the hardware-off callback, starts DISARMED, and cuts a new epoch.
 * Run on every notification AND on fixed periodic deadlines. GP_PENDING is
 * normal. Run rechecks faults/freshness even before the next output deadline.
 * Recover is an explicit local acknowledgement, never a CAN arm command.
 * It flushes old candidates again; a new neutral/ARM edge is still required. */
GpResult GpMotor_Init(GpMotorTask *, GpControl *,
                      const CommandMapConfig *, const MotorDriverConfig *);
/* Run the real fault supervisor with no motor parameters or driver initialized.
 * This mode always asserts dependency failure and can NEVER arm/recover.
 * It is for actual inhibited boards, not a simulated healthy safety circuit. */
GpResult GpMotor_InitInhibited(GpMotorTask *, GpControl *,
                              void (*off)(void *), void *user);
GpResult GpMotor_Run(GpMotorTask *);
GpResult GpMotor_Recover(GpMotorTask *);

#endif
