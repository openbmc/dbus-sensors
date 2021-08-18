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
            default_deps,
            gpiodcxx,
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
            default_deps,
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
            default_deps,
            gpiodcxx,
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
            default_deps,
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
            default_deps,
            gpiodcxx,
            i2c,
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
            default_deps,
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
            default_deps,
            i2c,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('nvme').enabled()
    nvme_srcs = files('NVMeSensorMain.cpp', 'NVMeSensor.cpp')
    nvme_srcs += files('NVMeBasicContext.cpp')

    nvme_deps = [ default_deps, i2c, thresholds_dep, utils_dep, threads ]

    mi_mctp = get_option('nvme-mi-mctp')
    if mi_mctp.enabled()
        nvme_srcs += files('NVMeMCTPContext.cpp')
        nvme_deps += meson.get_compiler('cpp').find_library('libmctp')
        conf_data = configuration_data()
        conf_data.set('HAVE_NVME_MI_MCTP', true)
    endif

    executable(
        'nvmesensor',
        sources: nvme_srcs,
        dependencies: nvme_deps,
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
            default_deps,
            pwmsensor_dep,
            thresholds_dep,
            utils_dep,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

# boost dependency
boost_dep = dependency('boost')
# libgpiod dependency
libgpiod = dependency('libgpiod')
boost_cpp_args = [
  '-DBOOST_ASIO_DISABLE_THREADS',
  '-DBOOST_ALL_NO_LIB',
  '-DBOOST_COROUTINES_NO_DEPRECATION_WARNING',
  '-DBOOST_ERROR_CODE_HEADER_ONLY',
  '-DBOOST_NO_RTTI',
  '-DBOOST_NO_TYPEID',
  '-DBOOST_SYSTEM_NO_DEPRECATED',
  '-DBOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT',
]
if get_option('gpiosensor').enabled()
    executable(
        'gpiosensor',
        'GPIOSensor.cpp',
        'GPIOSensorMain.cpp',
        dependencies: [
            sdbusplus,
            boost_dep,
            libgpiod
        ],
        cpp_args: boost_cpp_args,
        implicit_include_directories: false,
        include_directories: '../include',
        install: true
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
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif
