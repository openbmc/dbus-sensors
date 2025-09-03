src_inc = include_directories('..')

executable(
    'intrusionsensor',
    'ChassisIntrusionSensor.cpp',
    'IntrusionSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, i2c, utils_dep],
    include_directories: src_inc,
    install: get_option('intrusion').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
