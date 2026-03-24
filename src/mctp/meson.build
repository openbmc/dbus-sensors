if not get_option('mctp').allowed()
    subdir_done()
endif

mctpreactor_lib = static_library(
    'mctpreactor',
    'MCTPReactorMain.cpp',
    'MCTPReactor.cpp',
    'MCTPEndpoint.cpp',
    dependencies: [default_deps, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [mctpreactor_lib],
    dependencies: [default_deps, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'mctpreactor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DMCTP']
