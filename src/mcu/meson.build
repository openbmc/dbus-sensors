src_inc = include_directories('..')

executable(
    'mcutempsensor',
    'MCUTempSensor.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        i2c,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: src_inc,
    install: true,
)
