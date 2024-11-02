phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'cablemonitor',
    'CableMonitor.cpp',
    'CableConfig.cpp',
    'CableEvent.cpp',
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
    ],
    install: true,
)
