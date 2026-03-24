if not get_option('ipmb').allowed()
    subdir_done()
endif

ipmbsensor_lib = static_library(
    'ipmbsensor',
    'IpmbSensorMain.cpp',
    'IpmbSensor.cpp',
    'IpmbSDRSensor.cpp',
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [ipmbsensor_lib],
    dependencies: [default_deps, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'ipmbsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DIPMB']
