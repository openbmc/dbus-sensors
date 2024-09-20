executable(
    'psusensor',
    'PSUEvent.cpp',
    'PSUSensor.cpp',
    'PSUSensorMain.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        pwmsensor_dep,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: ['.', '../'],
    install: true,
)