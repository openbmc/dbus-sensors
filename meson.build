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
    meson_version: '>=1.1',
)

# Enable io_uring for all daemons with below flags.
#    '-DBOOST_ASIO_HAS_IO_URING',
#    '-DBOOST_ASIO_DISABLE_EPOLL',
#    '-DBOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT',
# Note, there is currently an issue with CPUSensor when used in
# conjunction with io_uring. So it has not been changed to use
# random file access. But we'd like to enable it for all daemons.
# https://github.com/openbmc/dbus-sensors/issues/19

add_project_arguments(
    '-Wno-psabi',
    '-Wuninitialized',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ASIO_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    '-DBOOST_ALLOW_DEPRECATED_HEADERS',
    '-DBOOST_ASIO_HAS_IO_URING',
    '-DBOOST_ASIO_DISABLE_EPOLL',
    '-DBOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT',
    language: 'cpp',
)

gpiodcxx = dependency('libgpiodcxx',
    default_options: ['bindings=cxx'],
)

# i2c-tools doesn't ship a pkg-config file for libi2c
i2c = meson.get_compiler('cpp').find_library('i2c')

sdbusplus = dependency('sdbusplus', fallback: 'sdbusplus', include_type: 'system')

phosphor_logging_dep = dependency('phosphor-logging')
nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')

threads = dependency('threads')

boost = dependency('boost', version: '>=1.79.0', fallback: 'boost', include_type: 'system')

uring = dependency('liburing', fallback: ['liburing', 'uring'], include_type: 'system')

default_deps = [
    boost,
    nlohmann_json_dep,
    phosphor_logging_dep,
    sdbusplus,
    uring,
]

subdir('service_files')
subdir('src')

if not get_option('tests').disabled()
    subdir('tests')
endif
