src_inc = include_directories('..')
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
    install: get_option('cable-monitor').allowed(),
    install_dir: get_option('libexecdir') / 'dbus-sensors',
)
