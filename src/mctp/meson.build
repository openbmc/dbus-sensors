executable(
    'mctpreactor',
    'MCTPReactorMain.cpp',
    'MCTPReactor.cpp',
    'MCTPEndpoint.cpp',
    dependencies: [default_deps, utils_dep],
    install: get_option('mctp').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
