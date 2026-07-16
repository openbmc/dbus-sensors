if not get_option('cable-monitor').allowed()
    subdir_done()
endif
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

cable_monitor_args = [
    '-DCABLE_MONITOR_RECONCILE_DELAY_SECONDS=' + get_option(
        'cable-monitor-reconcile-delay-seconds',
    ).to_string(),
]

executable(
    'cablemonitor',
    'CableMonitor.cpp',
    'CableConfig.cpp',
    'CableEvents.cpp',
    cpp_args: cable_monitor_args,
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

