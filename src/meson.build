peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../include']
endif

if get_option('adc').enabled()
    executable(
        'adcsensor',
        'ADCSensor.cpp',
        'ADCSensorMain.cpp',
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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
        cpp_args : thread_disable_args,
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

if get_option('mcu-temp').enabled()
    executable(
        'mcutempsensor',
        'MCUTempSensor.cpp',
        cpp_args : thread_disable_args,
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
    executable(
        'nvmesensor',
        'NVMeSensorMain.cpp',
        'NVMeSensor.cpp',
        cpp_args : thread_disable_args,
        dependencies: [
            i2c,
            sdbusplus,
            mctp,
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

if get_option('psu').enabled()
    executable(
        'psusensor',
        'PSUEvent.cpp',
        'PSUSensor.cpp',
        'PSUSensorMain.cpp',
        dependencies: [
            sdbusplus,
            threads,
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
        cpp_args : thread_disable_args,
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
