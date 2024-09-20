conf_data = configuration_data()
conf_data.set10(
    'VALIDATION_UNSECURE_FEATURE',
    get_option('validate-unsecure-feature').allowed(),
)
conf_data.set10(
    'INSECURE_UNRESTRICTED_SENSOR_OVERRIDE',
    get_option('insecure-sensor-override').allowed(),
)
configure_file(
    input: 'dbus-sensor_config.h.in',
    output: 'dbus-sensor_config.h',
    configuration: conf_data,
)

thresholds_a = static_library(
    'thresholds_a',
    'Thresholds.cpp',
    dependencies: default_deps,
)

thresholds_dep = declare_dependency(
    link_with: [thresholds_a],
    dependencies: default_deps,
)

utils_a = static_library(
    'utils_a',
    [
        'FileHandle.cpp',
        'SensorPaths.cpp',
        'Utils.cpp',
    ],
    dependencies: default_deps,
)

utils_dep = declare_dependency(
    link_with: [utils_a],
    dependencies: [sdbusplus],
)

devicemgmt_a = static_library(
    'devicemgmt_a',
    [
        'DeviceMgmt.cpp',
    ],
    dependencies: default_deps,
)

devicemgmt_dep = declare_dependency(
    link_with: [devicemgmt_a],
    dependencies: default_deps,
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'PwmSensor.cpp',
    dependencies: [default_deps, thresholds_dep],
)

pwmsensor_dep = declare_dependency(
    link_with: [pwmsensor_a],
    dependencies: [default_deps, thresholds_dep],
)

if get_option('intel-cpu').allowed()
    peci_dep = dependency('libpeci', required: true)
endif

if get_option('adc').allowed()
    subdir('ADC')
endif

if get_option('intel-cpu').allowed()
    subdir('IntelCPU')
endif

if get_option('exit-air').allowed()
    subdir('ExitAir')
endif

if get_option('fan').allowed()
    subdir('Fan')
endif

if get_option('hwmon-temp').allowed()
    subdir('HwmonTemp')
endif

if get_option('intrusion').allowed()
    subdir('Intrusion')
endif

if get_option('ipmb').allowed()
    subdir('IPMB')
endif

if get_option('mcu').allowed()
    subdir('MCUTemp')
endif

if get_option('nvme').allowed()
    subdir('NVME')
endif

if get_option('psu').allowed()
    subdir('PSU')
endif

if get_option('external').allowed()
    subdir('External')
endif
