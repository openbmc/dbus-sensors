src_inc = ['..']
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    src_inc += ['../../include']
endif

executable(
    'intelcpusensor',
    'IntelCPUSensorMain.cpp',
    'IntelCPUSensor.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep, peci_dep],
    include_directories: src_inc,
    install: get_option('intel-cpu').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
