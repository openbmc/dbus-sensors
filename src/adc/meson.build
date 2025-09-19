if not get_option('adc').allowed()
    subdir_done()
endif
executable(
    'adcsensor',
    'ADCSensor.cpp',
    'ADCSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

