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
            gpiodcxx,
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('cpu').enabled()
    executable(
        'cpusensor',
        'CPUSensorMain.cpp',
        'CPUSensor.cpp',
        dependencies: [
            gpiodcxx,
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: [
            '../include'
        ] + peci_incdirs,
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('exit-air').enabled()
    executable(
        'exitairtempsensor',
        'ExitAirTempSensor.cpp',
        dependencies: [
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('fan').enabled()
    executable(
        'fansensor',
        'FanMain.cpp',
        'TachSensor.cpp',
        'PwmSensor.cpp',
        dependencies: [
            sdbusplus,
            gpiodcxx
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('hwmon-temp').enabled()
    executable(
        'hwmontempsensor',
        'HwmonTempMain.cpp',
        'HwmonTempSensor.cpp',
        dependencies: [
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('intrusion').enabled()
    executable(
        'intrusionsensor',
        'ChassisIntrusionSensor.cpp',
        'IntrusionSensorMain.cpp',
        dependencies: [
            i2c,
            sdbusplus,
            gpiodcxx,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            utils_a,
        ],
    )
endif

if get_option('ipmb').enabled()
    executable(
        'ipmbsensor',
        'IpmbSensor.cpp',
        dependencies: [
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('mcu').enabled()
    executable(
        'mcutempsensor',
        'MCUTempSensor.cpp',
        dependencies: [
            i2c,
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            utils_a,
            thresholds_a,
        ],
    )
endif

if get_option('nvme').enabled()
    nvme_srcs = files('NVMeSensorMain.cpp', 'NVMeSensor.cpp')
    nvme_srcs += files('NVMeBasicContext.cpp')
    nvme_srcs += files('NVMeMCTPContext.cpp')

    mctp = meson.get_compiler('cpp').find_library('libmctp')
    nvme_deps = [ i2c, sdbusplus, threads, mctp ]

    executable(
        'nvmesensor',
        sources: nvme_srcs,
        dependencies: nvme_deps,
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            utils_a,
            thresholds_a,
        ],
    )
endif

if get_option('psu').enabled()
    executable(
        'psusensor',
        'PSUEvent.cpp',
        'PSUSensor.cpp',
        'PSUSensorMain.cpp',
        dependencies: [
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            pwmsensor_a,
            thresholds_a,
            utils_a,
        ],
    )
endif

if get_option('external').enabled()
    executable(
        'externalsensor',
        'ExternalSensor.cpp',
        'ExternalSensorMain.cpp',
        dependencies: [
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
        link_with: [
            thresholds_a,
            utils_a,
        ],
    )
endif
