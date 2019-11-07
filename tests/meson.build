gtest = dependency('gtest', main: true, disabler: true, required: build_tests)
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
