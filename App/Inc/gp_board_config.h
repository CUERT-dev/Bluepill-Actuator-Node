#ifndef GP_BOARD_CONFIG_H
#define GP_BOARD_CONFIG_H

/* Physical routing is final: docs/hardware/final_pinout.png.
 * The current board has no MCU enable, interlock or E-stop readback. Active
 * waveforms require either unpowered signal validation or an explicit bench
 * commissioning candidate. Neither mode provides independent power cutoff. */
#ifdef GP_BOARD_APPROVAL_HEADER
#include GP_BOARD_APPROVAL_HEADER
#endif
#ifndef GP_ENABLE_CAN
#define GP_ENABLE_CAN 0
#endif
#ifndef GP_ENABLE_ACTUATION
#define GP_ENABLE_ACTUATION 0
#endif
#ifndef GP_UNPOWERED_SIGNAL_TEST
#define GP_UNPOWERED_SIGNAL_TEST 0
#endif
#ifndef GP_CONTROLLED_BENCH_ACTUATION
#define GP_CONTROLLED_BENCH_ACTUATION 0
#endif
#ifndef GP_BENCH_COMMISSIONING_APPROVED
#define GP_BENCH_COMMISSIONING_APPROVED 0
#endif
#ifndef GP_STEERING_EXTERNAL_READY_BEFORE_ARM
#define GP_STEERING_EXTERNAL_READY_BEFORE_ARM 0
#endif
#ifndef GP_ENABLE_WATCHDOG
#define GP_ENABLE_WATCHDOG 0
#endif
#ifndef GP_HAS_DECODER
#define GP_HAS_DECODER 1
#endif
#ifndef GP_PIN_MAP_APPROVED
#define GP_PIN_MAP_APPROVED 1
#endif
#ifndef GP_CAN_SETTINGS_APPROVED
#define GP_CAN_SETTINGS_APPROVED 0
#endif
#ifndef GP_CAN_BITRATE
#define GP_CAN_BITRATE 0u
#endif
#ifndef GP_CAN_PRESCALER
#define GP_CAN_PRESCALER 0u
#endif
#ifndef GP_CAN_BS1_TQ
#define GP_CAN_BS1_TQ 0u
#endif
#ifndef GP_CAN_BS2_TQ
#define GP_CAN_BS2_TQ 0u
#endif
#ifndef GP_CAN_SJW_TQ
#define GP_CAN_SJW_TQ 0u
#endif
#ifndef GP_OUTPUT_SETTINGS_CONFIGURED
#define GP_OUTPUT_SETTINGS_CONFIGURED 0
#endif
/* 0=unselected. Only 1=PB3/TIM2_CH2/L-PWM is implemented.
 * Final wiring does NOT establish whether L-PWM is forward.
 * PB4/R-PWM requires a reviewed new timer route; it is never silently swapped. */
#ifndef GP_THROTTLE_FORWARD_ROUTE
#define GP_THROTTLE_FORWARD_ROUTE 0u
#endif
#ifndef GP_THROTTLE_PWM_HZ
#define GP_THROTTLE_PWM_HZ 0u
#endif
#ifndef GP_THROTTLE_CAP_PM
#define GP_THROTTLE_CAP_PM 0u
#endif
#ifndef GP_STEERING_CAP_PM
#define GP_STEERING_CAP_PM 0u
#endif
#ifndef GP_THROTTLE_SLEW_PM_S
#define GP_THROTTLE_SLEW_PM_S 0u
#endif
#ifndef GP_STEERING_SLEW_PM_S
#define GP_STEERING_SLEW_PM_S 0u
#endif
#ifndef GP_MAX_STEP_HZ
#define GP_MAX_STEP_HZ 0u
#endif
#ifndef GP_STEP_PULSE_US
#define GP_STEP_PULSE_US 0u
#endif
#ifndef GP_STEP_LOW_US
#define GP_STEP_LOW_US 0u
#endif
#ifndef GP_DIR_SETUP_US
#define GP_DIR_SETUP_US 0u
#endif
#ifndef GP_ENABLE_SETUP_US
#define GP_ENABLE_SETUP_US 0u
#endif
#ifndef GP_REVERSAL_DEADTIME_MS
#define GP_REVERSAL_DEADTIME_MS 0u
#endif
/* -1 explicitly means unknown polarity. */
#ifndef GP_PWM_ACTIVE_HIGH
#define GP_PWM_ACTIVE_HIGH (-1)
#endif
#ifndef GP_STEP_ACTIVE_HIGH
#define GP_STEP_ACTIVE_HIGH (-1)
#endif
#ifndef GP_DIR_POSITIVE_HIGH
#define GP_DIR_POSITIVE_HIGH (-1)
#endif
#ifndef GP_REVERSE_INACTIVE_HIGH
#define GP_REVERSE_INACTIVE_HIGH (-1)
#endif
#ifndef GP_RESET_SAFE_APPROVED
#define GP_RESET_SAFE_APPROVED 0
#endif
#ifndef GP_WATCHDOG_SETTINGS_APPROVED
#define GP_WATCHDOG_SETTINGS_APPROVED 0
#endif
#ifndef GP_IWDG_PRESCALER
#define GP_IWDG_PRESCALER 0u
#endif
#ifndef GP_IWDG_RELOAD
#define GP_IWDG_RELOAD 0u
#endif
#ifndef GP_WATCHDOG_TASK_DEADLINE_MS
#define GP_WATCHDOG_TASK_DEADLINE_MS 0u
#endif
#ifndef GP_IWDG_LSI_MAX_HZ
#define GP_IWDG_LSI_MAX_HZ 0u
#endif

/* Software scheduling proposals, not measured acceptance limits. */
#define GP_MOTOR_PERIOD_MS 10u
#define GP_COMMAND_TIMEOUT_MS 200u
#define GP_STAGE_BUDGET_MS 40u
#define GP_MAPPER_MAX_DT_MS 40u
#define GP_CAN_IRQ_PRIORITY 6u
#define GP_CAN_ISR_DRAIN_BUDGET 3u
#define GP_MOTOR_STACK_WORDS 512u
#define GP_CAN_STACK_WORDS 384u

#if GP_ENABLE_CAN && (!GP_PIN_MAP_APPROVED || !GP_CAN_SETTINGS_APPROVED)
#error "CAN requires approved pin map and explicit CAN timing"
#endif
#if GP_ENABLE_CAN && (GP_CAN_BITRATE == 0 || GP_CAN_PRESCALER < 1 || GP_CAN_PRESCALER > 1024 || GP_CAN_BS1_TQ < 1 || GP_CAN_BS1_TQ > 16 || GP_CAN_BS2_TQ < 1 || GP_CAN_BS2_TQ > 8 || GP_CAN_SJW_TQ < 1 || GP_CAN_SJW_TQ > 4 || GP_CAN_SJW_TQ > GP_CAN_BS2_TQ)
#error "Invalid CAN timing parameters"
#endif
#if GP_UNPOWERED_SIGNAL_TEST && GP_CONTROLLED_BENCH_ACTUATION
#error "Select exactly one output mode: signal validation or controlled bench"
#endif
#if GP_ENABLE_ACTUATION && !GP_UNPOWERED_SIGNAL_TEST && !GP_CONTROLLED_BENCH_ACTUATION
#error "Powered outputs require an explicit controlled bench commissioning mode"
#endif
#if GP_CONTROLLED_BENCH_ACTUATION && (!GP_ENABLE_ACTUATION || !GP_ENABLE_CAN)
#error "Controlled bench actuation requires CAN and waveform outputs"
#endif
/* Physical verification is not a compiler dependency. These status values may
 * remain zero in a commissioning candidate; no fake healthy input is supplied.
 * Numerical, routing and protocol feasibility checks below still apply. */
#if GP_UNPOWERED_SIGNAL_TEST && !GP_ENABLE_ACTUATION
#error "Unpowered signal testing requires waveform outputs"
#endif
#if GP_ENABLE_ACTUATION && (!GP_ENABLE_CAN || !GP_HAS_DECODER || !GP_OUTPUT_SETTINGS_CONFIGURED)
#error "Actuation requires decoder, CAN and explicitly configured output settings"
#endif
#if GP_ENABLE_ACTUATION && GP_THROTTLE_FORWARD_ROUTE != 1
#error "Only the PB3/TIM2_CH2 route exists; PB4 needs an approved reassignment"
#endif
#if GP_ENABLE_ACTUATION && ((GP_PWM_ACTIVE_HIGH != 0 && GP_PWM_ACTIVE_HIGH != 1) || (GP_STEP_ACTIVE_HIGH != 0 && GP_STEP_ACTIVE_HIGH != 1) || (GP_DIR_POSITIVE_HIGH != 0 && GP_DIR_POSITIVE_HIGH != 1) || (GP_REVERSE_INACTIVE_HIGH != 0 && GP_REVERSE_INACTIVE_HIGH != 1))
#error "Actuation requires explicit electrical polarity"
#endif
#if GP_ENABLE_ACTUATION && (GP_THROTTLE_CAP_PM < 1 || GP_THROTTLE_CAP_PM > 1000 || GP_STEERING_CAP_PM < 1 || GP_STEERING_CAP_PM > 1000 || GP_THROTTLE_SLEW_PM_S < 1 || GP_THROTTLE_SLEW_PM_S > 65535 || GP_STEERING_SLEW_PM_S < 1 || GP_STEERING_SLEW_PM_S > 65535 || GP_THROTTLE_PWM_HZ == 0 || GP_MAX_STEP_HZ == 0 || GP_STEP_PULSE_US < 1 || GP_STEP_PULSE_US > 65535 || GP_STEP_LOW_US < 1 || GP_STEP_LOW_US > 65535 || GP_DIR_SETUP_US < 1 || GP_DIR_SETUP_US > 65535 || GP_ENABLE_SETUP_US > 65535 || GP_REVERSAL_DEADTIME_MS == 0 || GP_REVERSAL_DEADTIME_MS >= 0x80000000u)
#error "Missing or out-of-range motor limits/timing (no truncated settings allowed)"
#endif
#if GP_ENABLE_WATCHDOG && (!GP_RESET_SAFE_APPROVED || !GP_WATCHDOG_SETTINGS_APPROVED)
#error "IWDG requires approved reset-safe outputs, timing and supervision policy"
#endif
/* Component ceilings/minima, NOT approved machine operating settings. See
 * docs/CURRENT_BOARD.md for signal testing and remaining electrical checks.
 * EM882S 2.5 us rounds upward to 3 us in this integer-us interface. */
#if GP_ENABLE_ACTUATION && (GP_THROTTLE_PWM_HZ > 25000u || GP_MAX_STEP_HZ > 200000u || GP_STEP_PULSE_US < 3u || GP_STEP_LOW_US < 3u || GP_DIR_SETUP_US < 5u)
#error "Motor timing exceeds documented BTS7960/EM882S component constraints"
#endif
#endif
