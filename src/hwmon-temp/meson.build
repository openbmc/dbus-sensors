if not get_option('hwmon-temp').allowed()
    subdir_done()
endif

hwmontemp_lib = static_library(
    'hwmontemp',
    'HwmonTempMain.cpp',
    'HwmonTempSensor.cpp',
    dependencies: [default_deps, devicemgmt_dep, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [hwmontemp_lib],
    dependencies: [default_deps, devicemgmt_dep, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'hwmontempsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DHWMONTEMP']
