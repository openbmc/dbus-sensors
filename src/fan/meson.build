src_inc = include_directories('..')

executable(
    'fansensor',
    'FanMain.cpp',
    'PresenceGpio.cpp',
    'TachSensor.cpp',
    '../PwmSensor.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        gpiodcxx,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: src_inc,
    install: true,
)
