if get_option('external').allowed()

    executable(
        'externalsensor',
        'ExternalSensor.cpp',
        'ExternalSensorMain.cpp',
        dependencies: [default_deps, thresholds_dep, utils_dep],
        include_directories: src_inc,
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
