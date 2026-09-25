set(component_sources
    "main.c"
    "rs02_control.c"
    "rs02_motion_config.c"
    "rs02_protocol.c"
    "rs02_twai.c"
    "rs02_twai_timing.c"
)
if(CONFIG_RS02_RUN_WEB_CONSOLE)
    list(APPEND component_sources "rs02_web_console.c")
endif()

# ESP-IDF resolves component requirements before project Kconfig values are
# available, so keep the optional web console dependencies unconditional.
idf_component_register(
    SRCS ${component_sources}
    INCLUDE_DIRS "."
    REQUIRES driver esp_system esp_timer
    PRIV_REQUIRES esp_event esp_http_server esp_netif esp_wifi json nvs_flash
    EMBED_TXTFILES "web/index.html"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
