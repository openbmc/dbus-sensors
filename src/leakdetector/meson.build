if not get_option('leakdetector').allowed()
    subdir_done()
endif
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'leakdetector',
    'LeakDetectionManager.cpp',
    'LeakGPIODetector.cpp',
    'LeakEvents.cpp',
    systemdinterface_src,
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        gpiodcxx,
        gpiinterface_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
