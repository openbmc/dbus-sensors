src_inc = include_directories('..')

executable(
    'exitairtempsensor',
    'ExitAirTempSensor.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
