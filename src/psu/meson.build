if not get_option('psu').allowed()
    subdir_done()
endif

psusensor_lib = static_library(
    'psusensor',
    'PSUEvent.cpp',
    'PSUSensor.cpp',
    'PSUSensorMain.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        pwmsensor_dep,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [psusensor_lib],
    dependencies: [
        default_deps,
        devicemgmt_dep,
        pwmsensor_dep,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: src_inc,
)
install_symlink(
    'psusensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DPSU']

