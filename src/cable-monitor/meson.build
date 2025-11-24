if not get_option('cable-monitor').allowed()
    subdir_done()
endif
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
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
    install: true,
    install_dir: get_option('libexecdir') / 'dbus-sensors',
)

