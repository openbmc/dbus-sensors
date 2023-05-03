conf_data = configuration_data()
conf_data.set10('VALIDATION_UNSECURE_FEATURE', get_option('validate-unsecure-feature').enabled())
conf_data.set10('INSECURE_UNRESTRICTED_SENSOR_OVERRIDE', get_option('insecure-sensor-override').enabled())
configure_file(input: '../src/dbus-sensor_config.h.in',
               output: 'dbus-sensor_config.h',
               configuration: conf_data)

gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: false)
    if gtest_proj.found()
        gtest_dep = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ]
        )
        gmock_dep = gtest_proj.dependency('gmock')
    else
        assert(
            not get_option('tests').enabled(),
            'Googletest is required if tests are enabled'
        )
    endif
endif

ut_deps_list = [
    gtest_dep,
    default_deps,
    thresholds_dep,
    utils_dep,
    gpiodcxx,
]

ut_deps_list += default_deps

test(
    'test_dbus_sensors',
    executable(
        'test_dbus_sensors',
        'test_main.cpp',

        'test_HwmonTempSensor.cpp',
        '../src/HwmonTempSensor.cpp',

        'test_TachSensor.cpp',
        '../src/TachSensor.cpp',

        'test_Threshold.cpp',

        'test_Utils.cpp',

        dependencies: ut_deps_list,
        cpp_args: uring_args,
    )
)


