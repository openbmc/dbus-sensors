if not get_option('adc').allowed()
    subdir_done()
endif

adcsensor_lib = static_library(
    'adcsensor',
    'ADCSensor.cpp',
    'ADCSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
)
dbus_sensors_deps += declare_dependency(
    link_with: [adcsensor_lib],
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
)
install_symlink(
    'adcsensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DADC']
