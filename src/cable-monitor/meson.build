if not get_option('cable-monitor').allowed()
    subdir_done()
endif
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

cablemonitor_lib = static_library(
    'cablemonitor',
    'CableMonitor.cpp',
    'CableConfig.cpp',
    'CableEvents.cpp',
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        notifywatch_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [cablemonitor_lib],
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        notifywatch_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
)
install_symlink(
    'cablemonitor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DCABLEMONITOR']