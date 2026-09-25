cmake_minimum_required(VERSION 3.16)

project(cybergear_protocol_tests C)

enable_testing()

add_executable(
    cybergear_protocol_tests
    test_cybergear_protocol.c
    ../main/cybergear_protocol.c
)

target_include_directories(cybergear_protocol_tests PRIVATE ../main)
target_compile_features(cybergear_protocol_tests PRIVATE c_std_11)
target_compile_options(
    cybergear_protocol_tests
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
)

if(NOT MSVC)
    target_link_libraries(cybergear_protocol_tests PRIVATE m)
endif()

add_test(NAME cybergear_protocol_tests COMMAND cybergear_protocol_tests)

add_executable(cybergear_motion_tests test_cybergear_motion.c ../main/cybergear_motion.c)
target_include_directories(cybergear_motion_tests PRIVATE ../main)
target_compile_features(cybergear_motion_tests PRIVATE c_std_11)
target_compile_options(cybergear_motion_tests PRIVATE -Wall -Wextra -Werror -Wpedantic)
if(NOT MSVC)
    target_link_libraries(cybergear_motion_tests PRIVATE m)
endif()
add_test(NAME cybergear_motion_tests COMMAND cybergear_motion_tests)

add_executable(
    cybergear_twai_timing_tests
    test_cybergear_twai_timing.c
    ../main/cybergear_twai_timing.c
)
target_include_directories(cybergear_twai_timing_tests PRIVATE ../main)
target_compile_features(cybergear_twai_timing_tests PRIVATE c_std_11)
target_compile_options(
    cybergear_twai_timing_tests
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
)
add_test(NAME cybergear_twai_timing_tests COMMAND cybergear_twai_timing_tests)
