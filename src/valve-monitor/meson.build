if not get_option('valve-monitor').allowed()
    subdir_done()
endif

src_inc = include_directories('..')
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'valvemonitor',
    'ValveMonitor.cpp',
    'AnalogValve.cpp',
    'BaseValve.cpp',
    'GPIOValve.cpp',
    'ValveFactory.cpp',
    'ValveEvents.cpp',
    'LocalConfig.cpp',
    systemdinterface_src,
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        gpiodcxx,
        gpiinterface_dep,
        gpointerface_dep,
        entitymanagerinterface_dep,
        notifywatch_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
