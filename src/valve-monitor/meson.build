if not get_option('valve-monitor').allowed()
    subdir_done()
endif

src_inc = include_directories('..')
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'valvemonitor',
    'ValveMonitor.cpp',
    'GPIOValve.cpp',
    'ValveEvents.cpp',
    dependencies: [
        default_deps,
        utils_dep,
        phosphor_dbus_interfaces_dep,
        gpiodcxx,
        gpiointerface_dep,
        entitymanagerinterface_dep,
    ],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
