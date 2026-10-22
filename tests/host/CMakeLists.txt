cmake_minimum_required(VERSION 3.22)
project(gp_host_tests LANGUAGES C)
enable_testing()
add_executable(gp_pipeline_tests test_pipeline.c)
target_link_libraries(gp_pipeline_tests PRIVATE gp_control gp_actuator gp_can_codec gp_hardware)
target_compile_options(gp_pipeline_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
foreach(case IN ITEMS commissioning_range axes readiness_stop reversal_failure readiness_slew readiness_fractional pending_fractional_neutral)
    add_test(NAME pipeline_${case} COMMAND gp_pipeline_tests ${case})
endforeach()
add_executable(gp_service_tests test_service.c ../../App/Src/gp_service.c)
target_include_directories(gp_service_tests PRIVATE ../../App/Inc)
target_compile_options(gp_service_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
add_test(NAME service_parser COMMAND gp_service_tests)

add_subdirectory(../../lib gp_actuator)
add_executable(gp_can_frame gp_can_frame.c)
target_link_libraries(gp_can_frame PRIVATE gp_can_codec)
target_compile_options(gp_can_frame PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME service_encoder_vector COMMAND gp_can_frame 1 600 -200 7)
set_tests_properties(service_encoder_vector PROPERTIES PASS_REGULAR_EXPRESSION "105#0100580238FF0762")
add_executable(gp_codec_supplied_tests test_gp_can_codec.c)
target_link_libraries(gp_codec_supplied_tests PRIVATE gp_can_codec)
add_executable(gp_codec_tests test_codec_validation.c)
target_link_libraries(gp_codec_tests PRIVATE gp_can_codec)
set_target_properties(gp_codec_supplied_tests gp_codec_tests gp_can_codec PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    foreach(target IN ITEMS gp_codec_supplied_tests gp_codec_tests gp_can_codec)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -Wpedantic)
    endforeach()
    # Supplied test uses assert; keep it active in every host build type.
    target_compile_options(gp_codec_supplied_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME codec_supplied COMMAND gp_codec_supplied_tests)
foreach(case IN ITEMS layout crc malformed limits flags output_preserved)
    add_test(NAME codec_${case} COMMAND gp_codec_tests ${case})
endforeach()
add_executable(gp_tests test_actuator.c headers_reverse.c)
target_link_libraries(gp_tests PRIVATE gp_actuator)
set_target_properties(gp_tests gp_actuator PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gp_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
    target_compile_options(gp_actuator PRIVATE -Wall -Wextra -Werror -Wpedantic)
endif()

foreach(case IN ITEMS map_endpoints map_fractional map_reversal map_flags
        map_invalid map_reset map_config map_large_dt driver_config
        driver_stop driver_safety driver_limits driver_errors driver_reversal
        driver_pending driver_rollover driver_alias combined)
    add_test(NAME ${case} COMMAND gp_tests ${case})
endforeach()

find_package(Python3 COMPONENTS Interpreter REQUIRED)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND (WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Linux"))
    # Native execution of the actual register-only shutdown against mapped storage.
    # No HAL/RTOS stub or synthetic hardware provider is required.
    add_executable(gp_bsp_shutdown_tests test_bsp_shutdown.c ../../App/Src/gp_output_shutdown.c)
    target_include_directories(gp_bsp_shutdown_tests PRIVATE ../.. ../../App/Inc ../../Core/Inc
        ../../lib/control/include ../../lib/hardware/include ../../lib/motor_driver/include
        ../../Drivers/STM32F1xx_HAL_Driver/Inc ../../Drivers/CMSIS/Include
        ../../Drivers/CMSIS/Device/ST/STM32F1xx/Include)
    target_compile_definitions(gp_bsp_shutdown_tests PRIVATE STM32F103xB USE_HAL_DRIVER
        HAL_CAN_MODULE_ENABLED HAL_TIM_MODULE_ENABLED HAL_IWDG_MODULE_ENABLED
        GP_ENABLE_ACTUATION=1 GP_ENABLE_CAN=1 GP_CONTROLLED_BENCH_ACTUATION=1
        GP_BOARD_APPROVAL_HEADER="App/Profiles/bench_actuation.h")
    target_compile_options(gp_bsp_shutdown_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -Wno-int-to-pointer-cast)
    add_test(NAME bsp_register_shutdown COMMAND gp_bsp_shutdown_tests)
endif()
foreach(case IN ITEMS default synthetic unapproved_can missing_decoder unknown_polarity
        truncated_timing unapproved_watchdog component_timing bench_profile bench_forbids_actuation
        powered_denied signal_profile signal_watchdog signal_missing_mode
        controlled_bench controlled_bench_unapproved controlled_bench_no_readiness
        controlled_bench_both_modes controlled_bench_template
        controlled_bench_zero_cap controlled_bench_zero_rate)
    add_test(NAME config_${case} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test_board_gates.py
        --cc ${CMAKE_C_COMPILER} --root ${CMAKE_CURRENT_SOURCE_DIR}/../.. --case ${case})
endforeach()

add_executable(gp_hardware_tests test_hardware.c)
target_link_libraries(gp_hardware_tests PRIVATE gp_hardware gp_actuator)
target_include_directories(gp_hardware_tests PRIVATE ../../App/Inc)
set_target_properties(gp_hardware_tests gp_hardware PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gp_hardware_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
    target_compile_options(gp_hardware PRIVATE -Wall -Wextra -Werror -Wpedantic)
endif()
foreach(case IN ITEMS timer_plans can_timing setup_pending cancellation errors
        microsecond_rollover watchdog scheduling driver_integration default_inhibit fault_during_setup
        neutral_axes power_readiness)
    add_test(NAME hardware_${case} COMMAND gp_hardware_tests ${case})
endforeach()

add_executable(gp_control_tests test_control.c)
target_link_libraries(gp_control_tests PRIVATE gp_control gp_actuator gp_can_codec)
set_target_properties(gp_control_tests gp_control PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gp_control_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
    target_compile_options(gp_control PRIVATE -Wall -Wextra -Werror -Wpedantic)
endif()
foreach(case IN ITEMS startup throttle steering ordinary_neutral stop_throttle
        stop_arm_burst stop_then_recovery_queued raw_full_stop event_full_stop
        duplicate sequence_wrap out_of_order stale_raw stale_event timestamp_capture
        timeout deadman arm_edge_order estop bus_off sender_reset driver_error
        pending fault_pending urgent_notification backlog event_budget tick_wrap
        repeated_shutdown recover_barrier inflight_generation inflight_backlog
        final_commit_fault final_commit_frame scheduled_notifications malformed
        missing_dependencies hardware_backlog deadline bounded_rx config_arguments
        hardware_barrier pending_reversal neutral_steering neutral_throttle
        neutral_both neutral_due neutral_coalesced neutral_fractional neutral_pending
        inhibited_supervisor diagnostic_counters unwired_estop unknown_estop_mode
        neutral_driver_error neutral_final_fault codec_path codec_rejected_freshness
        codec_sequence codec_stop_barrier codec_fault_recovery codec_neutral)
    add_test(NAME control_${case} COMMAND gp_control_tests ${case})
endforeach()
