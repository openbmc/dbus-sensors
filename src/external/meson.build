if not get_option('external').allowed()
    subdir_done()
endif
executable(
    'externalsensor',
    'ExternalSensor.cpp',
    'ExternalSensorMain.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

