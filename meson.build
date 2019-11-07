project(
    'dbus-sensors',
    'cpp',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++17'
    ],
    license: 'Apache-2.0',
    version: '0.1',
)

add_project_arguments(
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    language: 'cpp',
)

build_tests = get_option('tests')
cppfs = meson.get_compiler('cpp').find_library('stdc++fs')
gpiodcxx = dependency('libgpiodcxx')
i2c = meson.get_compiler('cpp').find_library('i2c')
sdbusplus = dependency('sdbusplus')
systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_pkgconfig_variable(
    'systemdsystemunitdir',
    define_variable: ['prefix', get_option('prefix')])
threads = dependency('threads')

thresholds_a = static_library(
    'thresholds_a',
    'src/Thresholds.cpp',
    implicit_include_directories: false,
    include_directories: 'include',
)

utils_a = static_library(
    'utils_a',
    'src/Utils.cpp',
    dependencies: [
        cppfs,
    ],
    implicit_include_directories: false,
    include_directories: 'include',
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'src/PwmSensor.cpp',
    implicit_include_directories: false,
    include_directories: 'include',
)

subdir('service_files')
subdir('src')

if not build_tests.disabled()
    subdir('tests')
endif
