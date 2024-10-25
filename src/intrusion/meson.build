src_inc = include_directories('..')

executable(
    'intrusionsensor',
    'ChassisIntrusionSensor.cpp',
    'IntrusionSensorMain.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        gpiodcxx,
        i2c,
        utils_dep,
    ],
    include_directories: src_inc,
    install: true,
)
