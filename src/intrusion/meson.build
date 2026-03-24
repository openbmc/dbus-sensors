if not get_option('intrusion').allowed()
    subdir_done()
endif

intrusionsensor_lib = static_library(
    'intrusionsensor',
    'ChassisIntrusionSensor.cpp',
    'IntrusionSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, i2c, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [intrusionsensor_lib],
    dependencies: [default_deps, gpiodcxx, i2c, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'intrusionsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DINTRUSION']
