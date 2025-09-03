src_inc = include_directories('..')

executable(
    'fansensor',
    'FanMain.cpp',
    'PresenceGpio.cpp',
    'TachSensor.cpp',
    '../PwmSensor.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: get_option('fan').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
