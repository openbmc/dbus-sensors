src_inc = include_directories('..')

executable(
    'smbpbisensor',
    'SmbpbiSensor.cpp',
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: get_option('smbpbi').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
