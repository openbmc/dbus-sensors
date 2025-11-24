if get_option('mctp').allowed()

    executable(
        'mctpreactor',
        'MCTPReactorMain.cpp',
        'MCTPReactor.cpp',
        'MCTPEndpoint.cpp',
        dependencies: [default_deps, utils_dep],
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
