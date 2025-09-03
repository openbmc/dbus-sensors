src_inc = include_directories('..')

executable(
    'externalsensor',
    'ExternalSensor.cpp',
    'ExternalSensorMain.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: get_option('external').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
