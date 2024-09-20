executable(
    'fansensor',
    'FanMain.cpp',
    'TachSensor.cpp',
    '../PwmSensor.cpp',
    dependencies: [
        default_deps,
        gpiodcxx,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: ['.', '../'],
    install: true,
)