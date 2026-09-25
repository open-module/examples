idf_component_register(
    SRCS
        "main.c"
        "cybergear_motion.c"
        "cybergear_protocol.c"
        "cybergear_twai.c"
        "cybergear_twai_timing.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_system esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
