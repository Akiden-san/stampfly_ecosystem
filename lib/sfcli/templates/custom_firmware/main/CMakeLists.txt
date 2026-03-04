idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        sf_hal_imu
        sf_hal_baro
        sf_hal_tof
        sf_hal_flow
        sf_hal_motor
        sf_hal_led
        sf_hal_power
        sf_svc_state
        sf_svc_comm
        sf_algo_estimation
)
