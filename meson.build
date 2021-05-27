project(
    'dbus-sensors',
    'cpp',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++20'
    ],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=0.57.0',
)

add_project_arguments(
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    '-DBOOST_ALLOW_DEPRECATED_HEADERS',
    language: 'cpp',
)

build_tests = get_option('tests')
gpiodcxx = dependency('libgpiodcxx')

# i2c-tools doesn't ship a pkg-config file for libi2c
i2c = meson.get_compiler('cpp').find_library('i2c')

sdbusplus = dependency(
    'sdbusplus',
    fallback: [
        'sdbusplus',
        'sdbusplus_dep'
    ],
)

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    pkgconfig: 'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')])
threads = dependency('threads')

thresholds_a = static_library(
    'thresholds_a',
    'src/Thresholds.cpp',
    dependencies: [ sdbusplus ],
    implicit_include_directories: false,
    include_directories: 'include',
)

utils_a = static_library(
    'utils_a',
    ['src/Utils.cpp', 'src/SensorPaths.cpp'],
    dependencies: [ sdbusplus ],
    implicit_include_directories: false,
    include_directories: 'include',
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'src/PwmSensor.cpp',
    dependencies: [ sdbusplus ],
    implicit_include_directories: false,
    include_directories: 'include',
)


subdir('service_files')
subdir('src')

if not build_tests.disabled()
    subdir('tests')
endif
