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

have_boost_dbus = meson.get_compiler('cpp').has_header('dbus/connection.hpp')
if have_boost_dbus
test(
    'test_hwmon_temp_sensor',
    executable(
        'test_hwmon_temp_sensor',
        'test_HwmonTempSensor.cpp',
        dependencies: [
            gtest,
            boost_dbus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
    )
)

test(
    'test_tach_sensor',
    executable(
        'test_tach_sensor',
        'test_TachSensor.cpp',
        dependencies: [
            gtest,
            boost_dbus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
    )
)
endif


test(
    'test_sensor_paths',
    executable(
        'test_sensor_paths',
        'test_SensorPaths.cpp'
        dependencies: [
            gtest,
            sdbusplus,
        ],
        implicit_include_directories: false,
        include_directories: '../include',
    )
)
