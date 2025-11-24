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
    ['FileHandle.cpp', 'SensorPaths.cpp', 'Utils.cpp'],
    dependencies: default_deps,
)

utils_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [utils_a],
    dependencies: [sdbusplus],
)

devicemgmt_a = static_library(
    'devicemgmt_a',
    ['DeviceMgmt.cpp'],
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

systemdinterface_src = files('SystemdInterface.cpp')

gpiointerface_a = static_library(
    'gpiointerface_a',
    'GPIOInterface.cpp',
    dependencies: [default_deps, gpiodcxx],
)

gpiointerface_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [gpiointerface_a],
    dependencies: default_deps,
)

entitymanagerinterface_a = static_library(
    'entitymanagerinterface_a',
    'EntityManagerInterface.cpp',
    dependencies: default_deps,
)

entitymanagerinterface_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [entitymanagerinterface_a],
    dependencies: default_deps,
)

notifywatch_a = static_library(
    'notifywatch_a',
    'NotifyWatch.cpp',
    dependencies: default_deps,
)

notifywatch_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [notifywatch_a],
    dependencies: default_deps,
)

src_inc = include_directories('.')

peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../include']
endif

if get_option('intel-cpu').allowed()
    peci_dep = dependency('libpeci', required: true)
endif

subdir('adc')
subdir('cable-monitor')
subdir('exit-air')
subdir('external')
subdir('fan')
subdir('hwmon-temp')
subdir('intel-cpu')
subdir('intrusion')
subdir('ipmb')
subdir('leakdetector')
subdir('mctp')
subdir('mcu')
subdir('nvidia-gpu')
subdir('nvme')
subdir('psu')
subdir('smbpbi')
subdir('tests')
