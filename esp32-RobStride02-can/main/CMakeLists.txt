idf_component_register(
    SRCS
        "main.c"
        "rs02_control.c"
        "rs02_protocol.c"
        "rs02_twai.c"
        "rs02_twai_timing.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_system esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
