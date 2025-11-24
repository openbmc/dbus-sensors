if not get_option('mctp').allowed()
    subdir_done()
endif

executable(
    'mctpreactor',
    'MCTPReactorMain.cpp',
    'MCTPReactor.cpp',
    'MCTPEndpoint.cpp',
    dependencies: [default_deps, utils_dep],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

