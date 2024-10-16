src_inc = include_directories('..')
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'leakdetector',
    'LeakDetectionManager.cpp',
    'LeakDetector.cpp',
    'LeakEvents.cpp',
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        systemd_dep,
        gpiodcxx,
    ],
    include_directories: src_inc,
    install: true,
)
