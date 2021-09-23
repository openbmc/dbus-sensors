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
    '-Wno-psabi',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    '-DBOOST_ALLOW_DEPRECATED_HEADERS',
    language: 'cpp',
)

cpp = meson.get_compiler('cpp')

build_tests = get_option('tests')
gpiodcxx = dependency(
    'libgpiodcxx',
    fallback: ['libgpiod', 'gpiodcxx_dep'],
    default_options: ['bindings=cxx'],
)

# i2c-tools doesn't ship a pkg-config file for libi2c
i2c = meson.get_compiler('cpp').find_library('i2c')

sdbusplus = dependency(
    'sdbusplus',
    fallback: [
        'sdbusplus',
        'sdbusplus_dep'
    ],
)

phosphor_logging_dep = dependency(
    'phosphor-logging',
    fallback: ['phosphor-logging', 'phosphor_logging_dep'],
)

if cpp.has_header('nlohmann/json.hpp')
    nlohmann_json = declare_dependency()
else
    subproject('nlohmann-json')
    nlohmann_json = declare_dependency(
        include_directories: include_directories(
            'subprojects/nlohmann-json/single_include',
            'subprojects/nlohmann-json/single_include/nlohmann',
        )
    )
endif

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    pkgconfig: 'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')])
threads = dependency('threads')

default_deps = [
    nlohmann_json,
    phosphor_logging_dep,
    sdbusplus,
]

thresholds_a = static_library(
    'thresholds_a',
    'src/Thresholds.cpp',
    dependencies: default_deps,
    implicit_include_directories: false,
    include_directories: 'include',
)

thresholds_dep = declare_dependency(
    link_with: [ thresholds_a ],
    dependencies: default_deps,
)

utils_a = static_library(
    'utils_a',
    ['src/Utils.cpp', 'src/SensorPaths.cpp'],
    dependencies: default_deps,
    implicit_include_directories: false,
    include_directories: 'include',
)

utils_dep = declare_dependency(
    link_with: [ utils_a ],
    dependencies: [ sdbusplus ],
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'src/PwmSensor.cpp',
    dependencies: [ default_deps, thresholds_dep ],
    implicit_include_directories: false,
    include_directories: 'include',
)

pwmsensor_dep = declare_dependency(
    link_with: [ pwmsensor_a ],
    dependencies: [ default_deps, thresholds_dep ],
)

subdir('include')
subdir('service_files')
subdir('src')

if not build_tests.disabled()
    subdir('tests')
endif
