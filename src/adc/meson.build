src_inc = include_directories('..')

executable(
    'adcsensor',
    'ADCSensor.cpp',
    'ADCSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
