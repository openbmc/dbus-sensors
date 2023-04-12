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
    meson_version: '>=0.58.0',
)

# Note, there is currently an issue with CPUSensor when used in conjunction
# with io_uring.  For the moment, we enable uring for all other daemons, but
# we'd like to enable it for all daemons.
# https://github.com/openbmc/dbus-sensors/issues/19
uring_args = [
    '-DBOOST_ASIO_HAS_IO_URING',
    '-DBOOST_ASIO_DISABLE_EPOLL',
    '-DBOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT',
]

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
    language: 'cpp',
)

cpp = meson.get_compiler('cpp')

build_tests = get_option('tests')
gpiodcxx = dependency('libgpiodcxx',
    default_options: ['bindings=cxx'],
)

# i2c-tools doesn't ship a pkg-config file for libi2c
i2c = meson.get_compiler('cpp').find_library('i2c')

sdbusplus = dependency('sdbusplus', required : false, include_type: 'system')
if not sdbusplus.found()
  sdbusplus_proj = subproject('sdbusplus', required: true)
  sdbusplus = sdbusplus_proj.get_variable('sdbusplus_dep')
  sdbusplus = sdbusplus.as_system('system')
endif

phosphor_logging_dep = dependency('phosphor-logging')

if cpp.has_header('nlohmann/json.hpp')
    nlohmann_json = declare_dependency()
else
    nlohmann_json = dependency('nlohmann_json')
endif

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')])
threads = dependency('threads')

boost = dependency('boost',version : '>=1.79.0', required : false, include_type: 'system')
if not boost.found()
  subproject('boost', required: false)
  boost_inc = include_directories('subprojects/boost_1_79_0/', is_system:true)
  boost  = declare_dependency(include_directories : boost_inc)
  boost = boost.as_system('system')
endif

uring = dependency('liburing', required : false, include_type: 'system')
if not uring.found()
  uring_proj = subproject('liburing', required: true)
  uring = uring_proj.get_variable('uring')
  uring = uring.as_system('system')
endif

default_deps = [
    boost,
    nlohmann_json,
    phosphor_logging_dep,
    sdbusplus,
    uring,
]

subdir('service_files')
subdir('src')

if not build_tests.disabled()
    subdir('tests')
endif
