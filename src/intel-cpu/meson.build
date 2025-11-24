if not get_option('intel-cpu').allowed()
    subdir_done()
endif

intel_cpu_inc = src_inc

if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    intel_cpu_inc = [src_inc, '../../include']
endif

executable(
    'intelcpusensor',
    'IntelCPUSensorMain.cpp',
    'IntelCPUSensor.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep, peci_dep],
    include_directories: intel_cpu_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

