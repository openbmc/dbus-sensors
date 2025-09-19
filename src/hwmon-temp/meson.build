if not get_option('hwmon-temp').allowed()
    subdir_done()
endif
executable(
    'hwmontempsensor',
    'HwmonTempMain.cpp',
    'HwmonTempSensor.cpp',
    dependencies: [default_deps, devicemgmt_dep, thresholds_dep, utils_dep],
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

