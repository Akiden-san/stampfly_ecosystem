idf_component_register(
    SRCS "serial_repl.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        console
        freertos
        sf_svc_console
        sf_svc_state
)
