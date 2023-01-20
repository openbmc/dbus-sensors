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
        implicit_include_directories: false,
        include_directories: '../include',
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
        cpp_args: uring_args,
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
        cpp_args: uring_args,
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
            devicemgmt_dep,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
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
        cpp_args: uring_args,
        implicit_include_directories: false,
        include_directories: '../include',
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
        cpp_args: uring_args,
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif

if get_option('nvme').enabled()
    nvme_srcs = files(
        'NVMeSensorMain.cpp',
        'NVMeSensor.cpp',
        'NVMeBasic.cpp',
        'NVMeSubsys.cpp',
        'NVMeMi.cpp',
        'NVMeController.cpp'
    )

    pdi_dep = dependency ('phosphor-dbus-interfaces')
    libnvme = dependency('libnvme', 
        required : true,
        fallback : ['libnvme', 'libnvme_dep'],
        include_type: 'system'
    )
    libnvme_mi = dependency('libnvme-mi', 
        required : true,
        fallback : ['libnvme', 'libnvme_mi_dep'],
        include_type: 'system'
    )

    libnvme_vu = dependency('libnvme-interface', 
        required : true,
        fallback : ['libnvme-vu', 'nvme_intf_dep'],
        include_type: 'system'
    )

    nvme_deps = [ 
        default_deps, i2c, thresholds_dep, utils_dep, 
        threads, pdi_dep, libnvme, libnvme_mi,
        libnvme_vu
    ]

    executable(
        'nvmesensor',
        sources: nvme_srcs,
        dependencies: nvme_deps,
        cpp_args: uring_args + ['-frtti', '-UBOOST_ASIO_DISABLE_THREADS', '-UBOOST_ASIO_HAS_IO_URING'],
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
        cpp_args: uring_args,
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
            default_deps,
            thresholds_dep,
            utils_dep,
        ],
        cpp_args: uring_args,
        implicit_include_directories: false,
        include_directories: '../include',
        install: true,
    )
endif
