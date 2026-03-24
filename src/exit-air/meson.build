if not get_option('exit-air').allowed()
    subdir_done()
endif

exitairtempsensor_lib = static_library(
    'exitairtempsensor',
    'ExitAirTempSensor.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
)
dbus_sensors_deps += declare_dependency(
    link_with: [exitairtempsensor_lib],
    dependencies: [default_deps, thresholds_dep, utils_dep],
)
install_symlink(
    'exitairtempsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DEXITAIRTEMP']