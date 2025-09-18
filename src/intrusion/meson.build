
executable(
    'intrusionsensor',
    'ChassisIntrusionSensor.cpp',
    'IntrusionSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, i2c, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
