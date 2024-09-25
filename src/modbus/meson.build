src_inc = include_directories('..')

executable(
    'modbussensor',
    'ModbusSensor.cpp',
    dependencies: [
        default_deps,
        thresholds_dep,
        utils_dep,
        modbus_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec',
)
