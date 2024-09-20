executable(
    'hwmontempsensor',
    'HwmonTempMain.cpp',
    'HwmonTempSensor.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: ['.', '../'],
    install: true,
)