if not get_option('external').allowed()
    subdir_done()
endif

externalsensor_lib = static_library(
    'externalsensor',
    'ExternalSensor.cpp',
    'ExternalSensorMain.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [externalsensor_lib],
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'externalsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DEXTERNAL']
