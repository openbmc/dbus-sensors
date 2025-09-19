if not get_option('mcu').allowed()
    subdir_done()
endif

executable(
    'mcutempsensor',
    'MCUTempSensor.cpp',
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
