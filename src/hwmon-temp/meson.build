disable_host_power_monitoring = get_option('disable-host-power-monitoring')

hwmon_temp_cpp_args = []
if disable_host_power_monitoring
    hwmon_temp_cpp_args += ['-DDISABLE_HOST_POWER_MONITORING']
endif

src_inc = include_directories('..')

executable(
    'hwmontempsensor',
    'HwmonTempMain.cpp',
    'HwmonTempSensor.cpp',
    dependencies: [default_deps, devicemgmt_dep, thresholds_dep, utils_dep],
    include_directories: src_inc,
    cpp_args: hwmon_temp_cpp_args,
    install: true,
)
