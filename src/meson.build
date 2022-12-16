conf_data = configuration_data()
conf_data.set10('VALIDATION_UNSECURE_FEATURE', get_option('validate-unsecure-feature').enabled())
conf_data.set10('INSECURE_UNRESTRICTED_SENSOR_OVERRIDE', get_option('insecure-sensor-override').enabled())
configure_file(input: 'dbus-sensor_config.h.in',
               output: 'dbus-sensor_config.h',
               configuration: conf_data)

thresholds_a = static_library(
    'thresholds_a',
    'Thresholds.cpp',
    dependencies: default_deps,
)

thresholds_dep = declare_dependency(
    link_with: [ thresholds_a ],
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
    link_with: [ utils_a ],
    dependencies: [ sdbusplus ],
)

devicemgmt_a = static_library(
    'devicemgmt_a',
    [
        'DeviceMgmt.cpp',
    ],
    dependencies: default_deps,
)

devicemgmt_dep = declare_dependency(
    link_with: [ devicemgmt_a ],
    dependencies: default_deps,
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'PwmSensor.cpp',
    dependencies: [ default_deps, thresholds_dep ],
)

pwmsensor_dep = declare_dependency(
    link_with: [ pwmsensor_a ],
    dependencies: [ default_deps, thresholds_dep ],
)

peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../include']
endif

if get_option('adc').enabled()
    executable(
        'adcsensor',
        'ADCSensor.cpp',
        'ADCSensorMain.cpp',
        dependencies: [
            default_deps,
            gpiodcxx,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('intel-cpu').enabled()
    executable(
        'intelcpusensor',
        'IntelCPUSensorMain.cpp',
        'IntelCPUSensor.cpp',
        dependencies: [
            default_deps,
            gpiodcxx,
            thresholds_dep,
            utils_dep,
        ],
        include_directories: peci_incdirs,
        install: true,
    )
endif

if get_option('exit-air').enabled()
    executable(
        'exitairtempsensor',
        'ExitAirTempSensor.cpp',
        dependencies: [
            default_deps,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('fan').enabled()
    executable(
        'fansensor',
        'FanMain.cpp',
        'TachSensor.cpp',
        'PwmSensor.cpp',
        dependencies: [
            default_deps,
            gpiodcxx,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('hwmon-temp').enabled()
    executable(
        'hwmontempsensor',
        'HwmonTempMain.cpp',
        'HwmonTempSensor.cpp',
        dependencies: [
            default_deps,
            devicemgmt_dep,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('intrusion').enabled()
    executable(
        'intrusionsensor',
        'ChassisIntrusionSensor.cpp',
        'IntrusionSensorMain.cpp',
        dependencies: [
            default_deps,
            gpiodcxx,
            i2c,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('ipmb').enabled()
    executable(
        'ipmbsensor',
        'IpmbSensor.cpp',
        'IpmbSDRSensor.cpp',
        dependencies: [
            default_deps,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('mcu').enabled()
    executable(
        'mcutempsensor',
        'MCUTempSensor.cpp',
        dependencies: [
            default_deps,
            i2c,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('nvme').enabled()
    nvme_srcs = files('NVMeSensorMain.cpp', 'NVMeSensor.cpp')
    nvme_srcs += files('NVMeBasicContext.cpp')

    nvme_deps = [ default_deps, i2c, thresholds_dep, utils_dep, threads ]

    executable(
        'nvmesensor',
        sources: nvme_srcs,
        dependencies: nvme_deps,
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('psu').enabled()
    executable(
        'psusensor',
        'PSUEvent.cpp',
        'PSUSensor.cpp',
        'PSUSensorMain.cpp',
        dependencies: [
            default_deps,
            devicemgmt_dep,
            pwmsensor_dep,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif

if get_option('external').enabled()
    executable(
        'externalsensor',
        'ExternalSensor.cpp',
        'ExternalSensorMain.cpp',
        dependencies: [
            default_deps,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        install: true,
    )
endif
