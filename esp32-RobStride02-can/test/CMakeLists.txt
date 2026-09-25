cmake_minimum_required(VERSION 3.16)

project(rs02_protocol_tests C)

enable_testing()

add_executable(
    rs02_protocol_tests
    test_rs02_protocol.c
    ../main/rs02_protocol.c
)

target_include_directories(rs02_protocol_tests PRIVATE ../main)
target_compile_features(rs02_protocol_tests PRIVATE c_std_11)
target_compile_options(
    rs02_protocol_tests
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
)

if(NOT MSVC)
    target_link_libraries(rs02_protocol_tests PRIVATE m)
endif()

add_test(NAME rs02_protocol_tests COMMAND rs02_protocol_tests)

add_executable(rs02_control_tests test_rs02_control.c ../main/rs02_control.c)
target_include_directories(rs02_control_tests PRIVATE ../main)
target_compile_features(rs02_control_tests PRIVATE c_std_11)
target_compile_options(rs02_control_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
if(NOT MSVC)
    target_link_libraries(rs02_control_tests PRIVATE m)
endif()
add_test(NAME rs02_control_tests COMMAND rs02_control_tests)

add_executable(
    rs02_motion_config_tests
    test_rs02_motion_config.c
    ../main/rs02_motion_config.c
)
target_include_directories(rs02_motion_config_tests PRIVATE ../main)
target_compile_features(rs02_motion_config_tests PRIVATE c_std_11)
target_compile_options(
    rs02_motion_config_tests
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
)
if(NOT MSVC)
    target_link_libraries(rs02_motion_config_tests PRIVATE m)
endif()
add_test(NAME rs02_motion_config_tests COMMAND rs02_motion_config_tests)

add_executable(
    rs02_twai_timing_tests
    test_rs02_twai_timing.c
    ../main/rs02_twai_timing.c
)
target_include_directories(rs02_twai_timing_tests PRIVATE ../main)
target_compile_features(rs02_twai_timing_tests PRIVATE c_std_11)
target_compile_options(
    rs02_twai_timing_tests
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
)
add_test(NAME rs02_twai_timing_tests COMMAND rs02_twai_timing_tests)
