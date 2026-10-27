option(GP_ENABLE_CAN "Enable bxCAN using approved board timing" OFF)
option(GP_ENABLE_ACTUATION "Enable PWM/STEP waveform outputs" OFF)
option(GP_UNPOWERED_SIGNAL_TEST "Signal validation only: motor power MUST be disconnected" OFF)
option(GP_CONTROLLED_BENCH_ACTUATION "Real dual-axis commissioning candidate with actuation enabled" OFF)
option(GP_ENABLE_WATCHDOG "Start IWDG with approved reset-safe settings" OFF)
option(GP_DIRECT_COMMANDS "Apply valid commands without operator ARM/DEADMAN or neutral arming" OFF)
if(GP_DIRECT_COMMANDS AND (NOT GP_ENABLE_CAN OR NOT GP_ENABLE_ACTUATION))
    message(FATAL_ERROR "Direct commands require CAN and actuation enabled")
endif()
set(GP_BOARD_APPROVAL_HEADER "" CACHE FILEPATH "Reviewed header containing approved settings")
if(GP_BOARD_SOURCES)
    message(FATAL_ERROR "GP_BOARD_SOURCES is not supported by the current board; future hardware integrations require a reviewed revision")
endif()
if(GP_UNPOWERED_SIGNAL_TEST AND GP_CONTROLLED_BENCH_ACTUATION)
    message(FATAL_ERROR "Select exactly one output mode: signal validation or controlled bench")
endif()
if(GP_ENABLE_ACTUATION AND NOT GP_UNPOWERED_SIGNAL_TEST AND NOT GP_CONTROLLED_BENCH_ACTUATION)
    message(FATAL_ERROR "Powered outputs require an explicit controlled bench commissioning mode")
endif()
if(GP_CONTROLLED_BENCH_ACTUATION AND (NOT GP_ENABLE_ACTUATION OR NOT GP_ENABLE_CAN))
    message(FATAL_ERROR "Controlled bench actuation requires CAN and waveform outputs")
endif()
if(GP_UNPOWERED_SIGNAL_TEST AND (NOT GP_ENABLE_ACTUATION OR NOT GP_ENABLE_CAN OR GP_ENABLE_WATCHDOG))
    message(FATAL_ERROR "Unpowered signal testing requires CAN and waveform outputs, with watchdog OFF")
endif()
if(GP_DECODER_SOURCES OR GP_DECODER_INCLUDE_DIRS)
    message(FATAL_ERROR "The production CAN codec is built in; remove obsolete GP_DECODER_SOURCES/GP_DECODER_INCLUDE_DIRS to avoid duplicate providers")
endif()
if((GP_ENABLE_CAN OR GP_ENABLE_ACTUATION OR GP_ENABLE_WATCHDOG) AND NOT EXISTS "${GP_BOARD_APPROVAL_HEADER}")
    message(FATAL_ERROR "Enabled hardware requires GP_BOARD_APPROVAL_HEADER; no operational defaults are supplied")
endif()
if(GP_BOARD_APPROVAL_HEADER MATCHES "(^|[/\\\\])tests[/\\\\]")
    message(FATAL_ERROR "Test fixtures are not hardware approval headers and cannot configure a firmware image")
endif()
set(GP_HAS_DECODER 1)

set(GP_FREERTOS_ROOT ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
add_library(gp_freertos STATIC
    ${GP_FREERTOS_ROOT}/tasks.c
    ${GP_FREERTOS_ROOT}/queue.c
    ${GP_FREERTOS_ROOT}/list.c
    ${GP_FREERTOS_ROOT}/portable/GCC/ARM_CM3/port.c
)
target_include_directories(gp_freertos PUBLIC ${GP_FREERTOS_ROOT}/include
    ${GP_FREERTOS_ROOT}/portable/GCC/ARM_CM3 ${CMAKE_SOURCE_DIR}/App/Inc)
target_link_libraries(gp_freertos PUBLIC stm32cubemx)

add_library(gp_firmware OBJECT
    ${CMAKE_SOURCE_DIR}/App/Src/gp_app.c
    ${CMAKE_SOURCE_DIR}/App/Src/gp_stm32_bsp.c
    ${CMAKE_SOURCE_DIR}/App/Src/gp_output_shutdown.c
    ${CMAKE_SOURCE_DIR}/App/Src/gp_diag.c
    ${CMAKE_SOURCE_DIR}/App/Src/gp_diag_format.c
    ${CMAKE_SOURCE_DIR}/App/Src/gp_service.c
)
target_include_directories(gp_firmware PUBLIC ${CMAKE_SOURCE_DIR}/App/Inc)
target_link_libraries(gp_firmware PUBLIC gp_freertos gp_control gp_hardware gp_can_codec stm32cubemx)
target_compile_definitions(gp_firmware PUBLIC
    GP_ENABLE_CAN=$<BOOL:${GP_ENABLE_CAN}>
    GP_ENABLE_ACTUATION=$<BOOL:${GP_ENABLE_ACTUATION}>
    GP_UNPOWERED_SIGNAL_TEST=$<BOOL:${GP_UNPOWERED_SIGNAL_TEST}>
    GP_CONTROLLED_BENCH_ACTUATION=$<BOOL:${GP_CONTROLLED_BENCH_ACTUATION}>
    GP_ENABLE_WATCHDOG=$<BOOL:${GP_ENABLE_WATCHDOG}>
    GP_DIRECT_COMMANDS=$<BOOL:${GP_DIRECT_COMMANDS}>
    GP_HAS_DECODER=${GP_HAS_DECODER})
if(GP_BOARD_APPROVAL_HEADER)
    file(TO_CMAKE_PATH "${GP_BOARD_APPROVAL_HEADER}" approved_header)
    target_compile_definitions(gp_firmware PUBLIC GP_BOARD_APPROVAL_HEADER="${approved_header}")
endif()

# Keep additional HAL registrations outside the CubeMX-generated CMake file.
target_sources(STM32_Drivers PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c)
target_compile_definitions(stm32cubemx INTERFACE
    HAL_CAN_MODULE_ENABLED HAL_TIM_MODULE_ENABLED HAL_IWDG_MODULE_ENABLED)
target_link_libraries(${CMAKE_PROJECT_NAME} gp_firmware gp_freertos gp_hardware gp_can_codec)
# Keep both public codec entry points in the firmware API, even though this
# receive-only node does not currently call the encoder or transmit commands.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "LINKER:--undefined=GP_EncodeCan")

find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_program(GP_ARM_NM arm-none-eabi-nm REQUIRED)
find_program(GP_ARM_OBJDUMP arm-none-eabi-objdump REQUIRED)
add_custom_target(gp_check_enabled_syntax
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/firmware/test_enabled_syntax.py
        --cc ${CMAKE_C_COMPILER} --root ${CMAKE_SOURCE_DIR}
    VERBATIM)
set(profile_check_args)
if(NOT GP_ENABLE_CAN AND NOT GP_ENABLE_ACTUATION AND NOT GP_ENABLE_WATCHDOG)
    list(APPEND profile_check_args --inhibited)
endif()
if(NOT GP_ENABLE_ACTUATION)
    list(APPEND profile_check_args --motor-inhibited)
elseif(GP_UNPOWERED_SIGNAL_TEST)
    list(APPEND profile_check_args --signal-test)
else()
    list(APPEND profile_check_args --bench-actuation)
endif()
if(GP_ENABLE_CAN)
    list(APPEND profile_check_args --can-enabled)
endif()
set(commissioning_copy_command)
set(commissioning_manifest_args)
set(commissioning_hex)
if(GP_CONTROLLED_BENCH_ACTUATION)
    set(commissioning_hex "${CMAKE_BINARY_DIR}/CUERT-Bluepill-DualAxis-Commissioning.hex")
    list(APPEND commissioning_copy_command COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex" "${commissioning_hex}")
    list(APPEND commissioning_manifest_args --commissioning-hex "${commissioning_hex}")
endif()
add_custom_target(gp_firmware_artifacts ALL
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    ${commissioning_copy_command}
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/check_firmware.py
        --elf $<TARGET_FILE:${CMAKE_PROJECT_NAME}> --nm ${GP_ARM_NM}
        --objdump ${GP_ARM_OBJDUMP} ${profile_check_args}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/firmware_manifest.py
        --elf $<TARGET_FILE:${CMAKE_PROJECT_NAME}> --source ${CMAKE_SOURCE_DIR}
        --compiler ${CMAKE_C_COMPILER} --size ${CMAKE_SIZE}
        --can $<BOOL:${GP_ENABLE_CAN}> --actuation $<BOOL:${GP_ENABLE_ACTUATION}>
        --watchdog $<BOOL:${GP_ENABLE_WATCHDOG}> --decoder ${GP_HAS_DECODER}
        --unpowered-signal-test $<BOOL:${GP_UNPOWERED_SIGNAL_TEST}>
        --controlled-bench-actuation $<BOOL:${GP_CONTROLLED_BENCH_ACTUATION}>
        --direct-commands $<BOOL:${GP_DIRECT_COMMANDS}>
        ${commissioning_manifest_args}
        --build-type ${CMAKE_BUILD_TYPE}
    DEPENDS ${CMAKE_PROJECT_NAME}
    BYPRODUCTS ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
        ${commissioning_hex}
        ${CMAKE_BINARY_DIR}/firmware-manifest.json ${CMAKE_BINARY_DIR}/firmware.size.txt
        ${CMAKE_BINARY_DIR}/SHA.txt
    VERBATIM)
