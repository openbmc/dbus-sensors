src_inc = include_directories('..')

executable(
    'cablemonitor',
    'CableMonitor.cpp',
    'CableConfig.cpp',
    'CableEvents.cpp',
    dependencies: [
        default_deps,
        utils_dep,
        notifywatch_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: get_option('libexecdir') / 'dbus-sensors',
)
