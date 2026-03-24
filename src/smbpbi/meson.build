if not get_option('smbpbi').allowed()
    subdir_done()
endif

smbpbisensor_lib = static_library(
    'smbpbisensor',
    'SmbpbiSensor.cpp',
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [smbpbisensor_lib],
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'smbpbisensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DSMBPBI']
