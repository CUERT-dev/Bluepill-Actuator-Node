#include "gp_app.h"
#include "gp_board_config.h"
#include "gp_stm32_bsp.h"
#include "gp_diag.h"
#include "FreeRTOS.h"
#include "task.h"

_Static_assert(GP_CAN_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "CAN FromISR calls require a legal syscall interrupt priority");
_Static_assert(GP_CAN_IRQ_PRIORITY < (1u << configPRIO_BITS), "CAN priority out of range");
_Static_assert(configTICK_RATE_HZ == 1000u, "HAL/RTOS tick bridge requires 1 kHz");

static GpControl channels;
static GpCanRxTask can_rx;
static GpMotorTask motor;
static GpTaskWatchdog watchdog;
static StaticTask_t motor_tcb, can_tcb, idle_tcb;
static StackType_t motor_stack[GP_MOTOR_STACK_WORDS], can_stack[GP_CAN_STACK_WORDS];
static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
static TaskHandle_t motor_handle, can_handle;
static volatile bool recovery_requested;
static uint32_t recovery_count;
static GpResult recovery_result = GP_INVALID;

static uint32_t lock(void *user, bool isr)
{
    (void)user; (void)isr;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved;
}
static void unlock(void *user, bool isr, uint32_t saved)
{
    (void)user; (void)isr;
    __DMB();
    __set_PRIMASK(saved);
}
static uint32_t now_ms(void *user) { (void)user; return HAL_GetTick(); }
static void notify(void *user, GpWakeTarget target, bool isr)
{
    (void)user;
    TaskHandle_t handle = target == GP_WAKE_MOTOR ? motor_handle : can_handle;
    if (!handle) return;
    if (isr) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(handle, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xTaskNotifyGive(handle);
    }
}

static void motor_entry(void *unused)
{
    (void)unused;
    CommandMapConfig mapping;
    MotorDriverConfig driver;
    GpBsp_MotorOff(NULL);
    bool ready = GpBsp_MotorConfigure(&mapping, &driver) == GP_OK &&
                 GpMotor_Init(&motor, &channels, &mapping, &driver) == GP_OK;
    gp_boot_status.motor_ready = ready;
    if (!ready && GpMotor_InitInhibited(&motor, &channels, GpBsp_MotorOff, NULL) != GP_OK)
        GpApp_Fatal(8);
    /* CAN interrupts are enabled only after tasks, channels, and motor startup
     * have established their off state. Hardware approvals still gate startup. */
    if (GpBsp_CanStart() != GP_OK) gp_boot_status.blockers |= GP_BLOCK_CAN;
    watchdog.deadline_ms = GP_WATCHDOG_TASK_DEADLINE_MS;
    if (!GpBsp_WatchdogStart() && GP_ENABLE_WATCHDOG) {
        GpControl_LatchFault(&channels, GP_FAULT_DEPENDENCY, false);
    }
    uint32_t deadline = HAL_GetTick() + GP_MOTOR_PERIOD_MS;
    for (;;) {
        uint32_t now = HAL_GetTick();
        (void)GpBsp_Micros(); /* keep DWT extension serviced across cycle wraps */
        uint32_t saved = lock(NULL, false);
        bool recover = recovery_requested;
        recovery_requested = false;
        unlock(NULL, false, saved);
        (void)GpMotor_Run(&motor);
        if (recover) {
            ++recovery_count;
            recovery_result = GP_INVALID;
            if (motor.state == GP_FAULT_SHUTDOWN) {
                GpBsp_MotorOff(NULL);
                GpResult restarted = GpBsp_CanRestart();
                /* A failed restart must never acknowledge supervisor faults. */
                if (restarted == GP_OK) recovery_result = GpMotor_Recover(&motor);
                else recovery_result = restarted;
            }
        }
        saved = lock(NULL, false);
        watchdog.motor_ms = now;
        watchdog.motor_seen = true;
        bool feed = GpTaskWatchdog_Ready(&watchdog, now);
        unlock(NULL, false, saved);
        if (feed) GpBsp_WatchdogFeed();
        uint32_t wait = GpSchedule_Wait(HAL_GetTick(), GP_MOTOR_PERIOD_MS, &deadline);
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
    }
}

static void can_entry(void *unused)
{
    (void)unused;
    for (;;) {
        (void)GpCanRx_Run(&can_rx);
        uint32_t saved = lock(NULL, false);
        watchdog.can_ms = HAL_GetTick();
        watchdog.can_seen = true;
        unlock(NULL, false, saved);
        /* A bounded health wake also detects a stalled worker without traffic. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(GP_MOTOR_PERIOD_MS));
    }
}

void GpApp_Create(void)
{
    GpControlConfig config = GpControl_DefaultConfig();
    config.motor_period_ms = GP_MOTOR_PERIOD_MS;
    config.command_timeout_ms = GP_COMMAND_TIMEOUT_MS;
    config.stage_budget_ms = GP_STAGE_BUDGET_MS;
    GpControlPort port = {NULL, lock, unlock, now_ms, notify,
                          GpBsp_ControlInputs, GpBsp_DiscardRx};
    if (GpControl_Init(&channels, &config, &port) != GP_OK) GpApp_Fatal(1);
    if (GpCanRx_Init(&can_rx, &channels, GP_DecodeCan) != GP_OK) GpApp_Fatal(2);
    can_handle = xTaskCreateStatic(can_entry, "CAN-RX", GP_CAN_STACK_WORDS,
                                   NULL, 2, can_stack, &can_tcb);
    motor_handle = xTaskCreateStatic(motor_entry, "Motor", GP_MOTOR_STACK_WORDS,
                                     NULL, 3, motor_stack, &motor_tcb);
    if (!can_handle || !motor_handle) GpApp_Fatal(3);
    GpDiag_Start();
    vTaskStartScheduler();
    GpApp_Fatal(4);
}

GpResult GpApp_CaptureFrame(const GpCanFrame *frame, bool extended, bool remote)
{
    return GpControl_CaptureIsr(&channels, frame, extended, remote);
}
void GpApp_LatchCanFault(uint32_t reason)
{
    GpControl_LatchFault(&channels, reason, __get_IPSR() != 0);
}
void GpApp_RequestRecovery(void)
{
    uint32_t saved = lock(NULL, false);
    recovery_requested = true;
    unlock(NULL, false, saved);
    notify(NULL, GP_WAKE_MOTOR, false);
}
void GpApp_GetStatus(GpDiagStatus *out)
{
    if (!out) return;
    uint32_t saved = lock(NULL, false);
    *out = (GpDiagStatus){
        .uptime_ms = HAL_GetTick(), .blockers = gp_boot_status.blockers,
        .reset_flags = gp_boot_status.reset_flags, .panic = gp_boot_status.panic_reason,
        .state = motor.state, .faults = motor.latched_faults, .history = motor.fault_history,
        .captured = channels.captured, .dropped = channels.raw_dropped,
        .decoded = channels.decoded, .rejected = channels.rejected,
        .stale = channels.stale, .published = channels.published, .barred = channels.barred,
        .decoded_ms = channels.last_decoded_ms, .decoded_command = channels.last_decoded,
        .requested = motor.latest, .applied = motor.applied,
        .can_ready = gp_boot_status.can_ready, .motor_ready = gp_boot_status.motor_ready,
        .watchdog = gp_boot_status.watchdog_started, .recovery_requests = recovery_count,
        .recovery_result = recovery_result,
        .can_btr = gp_boot_status.can_ready ? CAN1->BTR : 0,
        .can_esr = gp_boot_status.can_ready ? CAN1->ESR : 0
    };
    unlock(NULL, false, saved);
}
void GpApp_KernelTick(void)
{
    extern void xPortSysTickHandler(void);
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) xPortSysTickHandler();
}
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *words)
{
    *tcb = &idle_tcb; *stack = idle_stack; *words = configMINIMAL_STACK_SIZE;
}
void GpApp_Fatal(uint32_t reason)
{
    __disable_irq();
    gp_boot_status.panic_reason = reason;
    /* A damaged task/kernel stack cannot safely call motor/RTOS callbacks.
     * Approved IWDG/reset-safe hardware must cover this failure class. */
#if GP_RESET_SAFE_APPROVED
    NVIC_SystemReset();
#endif
    for (;;) __WFI();
}
void GpApp_AssertFailed(const char *file, int line)
{
    (void)file;
    GpApp_Fatal(0x10000u | (uint32_t)line);
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task; (void)name;
    GpApp_Fatal(5);
}
