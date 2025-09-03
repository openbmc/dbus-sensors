src_inc = include_directories('..')

executable(
    'hwmontempsensor',
    'HwmonTempMain.cpp',
    'HwmonTempSensor.cpp',
    dependencies: [default_deps, devicemgmt_dep, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: get_option('hwmon-temp').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
