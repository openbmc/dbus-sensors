src_inc = include_directories('..')

executable(
    'leakdetector',
    'LeakDetectionManager.cpp',
    'LeakGPIODetector.cpp',
    'LeakEvents.cpp',
    systemdinterface_src,
    dependencies: [
        default_deps,
        utils_dep,
        gpiodcxx,
        gpiointerface_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
