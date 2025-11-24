if get_option('ipmb').allowed()

    executable(
        'ipmbsensor',
        'IpmbSensorMain.cpp',
        'IpmbSensor.cpp',
        'IpmbSDRSensor.cpp',
        dependencies: [default_deps, thresholds_dep, utils_dep],
        include_directories: src_inc,
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
