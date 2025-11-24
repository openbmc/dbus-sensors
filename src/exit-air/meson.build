if get_option('exit-air').allowed()

    executable(
        'exitairtempsensor',
        'ExitAirTempSensor.cpp',
        dependencies: [default_deps, thresholds_dep, utils_dep],
        include_directories: src_inc,
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
