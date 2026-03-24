if not get_option('mcu').allowed()
    subdir_done()
endif

mcutempsensor_lib = static_library(
    'mcutempsensor',
    'MCUTempSensor.cpp',
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [mcutempsensor_lib],
    dependencies: [default_deps, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
)
install_symlink(
    'mcutempsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DMCU']
