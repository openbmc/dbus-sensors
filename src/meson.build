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
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
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
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: [
            '../include'
        ] + peci_incdirs,
        install: true,
    )
endif

if get_option('exit-air').enabled()
    executable(
        'exitairtempsensor',
        'ExitAirTempSensor.cpp',
        dependencies: [
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
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
            gpiodcxx,
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('hwmon-temp').enabled()
    executable(
        'hwmontempsensor',
        'HwmonTempMain.cpp',
        'HwmonTempSensor.cpp',
        dependencies: [
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('intrusion').enabled()
    executable(
        'intrusionsensor',
        'ChassisIntrusionSensor.cpp',
        'IntrusionSensorMain.cpp',
        dependencies: [
            gpiodcxx,
            i2c,
            sdbusplus,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('ipmb').enabled()
    executable(
        'ipmbsensor',
        'IpmbSensor.cpp',
        dependencies: [
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('mcu').enabled()
    executable(
        'mcutempsensor',
        'MCUTempSensor.cpp',
        dependencies: [
            i2c,
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('nvme').enabled()
    mctp = meson.get_compiler('cpp').find_library('libmctp')
    executable(
        'nvmesensor',
        'NVMeSensorMain.cpp',
        'NVMeSensor.cpp',
        'NVMeMCTPContext.cpp',
        dependencies: [
            i2c,
            mctp,
            sdbusplus,
            thresholds_dep,
            utils_dep,
	    ],
        implicit_include_directories: false,
        include_directories: '../include',
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
            pwmsensor_dep,
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('external').enabled()
    executable(
        'externalsensor',
        'ExternalSensor.cpp',
        'ExternalSensorMain.cpp',
        dependencies: [
            sdbusplus,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif
