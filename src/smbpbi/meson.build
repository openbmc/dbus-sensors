phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')
src_inc = include_directories('..')

executable(
    'smbpbisensor',
    'main.cpp',
    dependencies: [
        default_deps,
        phosphor_dbus_interfaces_dep,
        phosphor_logging_dep,
        utils_dep,
    ],
    include_directories: src_inc,
    install: true,
)
