if not get_option('fan').allowed()
    subdir_done()
endif

executable(
    'fansensor',
    'FanMain.cpp',
    'PresenceGpio.cpp',
    'TachSensor.cpp',
    '../PwmSensor.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

