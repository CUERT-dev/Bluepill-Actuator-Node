add_library(gp_actuator OBJECT
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/command_map.c
    ${CMAKE_CURRENT_LIST_DIR}/motor_driver/src/motor_driver.c
)
target_include_directories(gp_actuator PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/motor_driver/include
)
target_compile_features(gp_actuator PUBLIC c_std_11)

# Production revision-10 codec; shared types come only from Core/Inc/gp_types.h.
add_library(gp_can_codec OBJECT can/src/gp_can_codec.c)
target_include_directories(gp_can_codec PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../Core/Inc)
target_compile_features(gp_can_codec PUBLIC c_std_11)

# Portable control only. No decoder, RTOS, CAN HAL or powered adapter fallback.
add_library(gp_control OBJECT control/src/gp_control.c)
target_include_directories(gp_control PUBLIC control/include)
target_link_libraries(gp_control PUBLIC gp_actuator)
target_compile_features(gp_control PUBLIC c_std_11)

add_library(gp_hardware OBJECT hardware/src/gp_hardware.c)
target_include_directories(gp_hardware PUBLIC hardware/include ../Core/Inc)
target_compile_features(gp_hardware PUBLIC c_std_11)
