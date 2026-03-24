if not get_option('leakdetector').allowed()
    subdir_done()
endif
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

leakdetector_lib = static_library(
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
        gpiointerface_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
)
dbus_sensors_deps += [declare_dependency(
    link_with: [leakdetector_lib],
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        gpiodcxx,
        gpiointerface_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
), phosphor_dbus_interfaces_dep]
install_symlink(
    'leakdetector',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DLEAKDETECTOR']
