# SITL default configuration
board_common px4_sitl_default

# Modules to include
set(config_module_list
    modules/commander
    modules/sensors
    modules/mavlink
    modules/ekf2
    modules/navigator
    modules/mc_att_control
    modules/mc_pos_control
    modules/land_detector
    modules/logger
    modules/fluidflow_simulator
)

# Driver modules
set(config_driver_list
    drivers_gyroscope
    drivers_accelerometer
    drivers_barometer
    drivers_magnetometer
)

# Explicitly enable your module
set(CONFIG_MODULES_FLUIDFLOW_SIMULATOR y)
