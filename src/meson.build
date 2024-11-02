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
    include_directories: ['.'],
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
    include_directories: ['.'],
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
    include_directories: ['.'],
    link_with: [devicemgmt_a],
    dependencies: default_deps,
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'PwmSensor.cpp',
    dependencies: [default_deps, thresholds_dep],
)

pwmsensor_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [pwmsensor_a],
    dependencies: [default_deps, thresholds_dep],
)

systemd_a = static_library(
    'systemd_a',
    'Systemd.cpp',
    dependencies: default_deps,
)

systemd_dep = declare_dependency(
    link_with: [systemd_a],
    dependencies: default_deps,
)

peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../include']
endif

if get_option('intel-cpu').allowed()
    peci_dep = dependency('libpeci', required: true)
    subdir('intel-cpu')
endif

if get_option('adc').allowed()
    subdir('adc')
endif

if get_option('exit-air').allowed()
    subdir('exit-air')
endif

if get_option('fan').allowed()
    subdir('fan')
endif

if get_option('hwmon-temp').allowed()
    subdir('hwmon-temp')
endif

if get_option('intrusion').allowed()
    subdir('intrusion')
endif

if get_option('ipmb').allowed()
    subdir('ipmb')
endif

if get_option('mctp').allowed()
    subdir('mctp')
endif

if get_option('mcu').allowed()
    subdir('mcu')
endif

if get_option('nvme').allowed()
    subdir('nvme')
endif

if get_option('psu').allowed()
    subdir('psu')
endif

if get_option('external').allowed()
    subdir('external')
endif

if get_option('tests').allowed()
    subdir('tests')
endif

if get_option('leakdetector').allowed()
    subdir('leakdetector')
endif

if get_option('cable-monitor').allowed()
    subdir('cable-monitor')
endif
