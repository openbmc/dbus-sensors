if not get_option('intel-cpu').allowed()
    subdir_done()
endif

intel_cpu_inc = [src_inc]
cxx = meson.get_compiler('cpp')
if not cxx.has_header('linux/peci-ioctl.h')
    intel_cpu_inc += include_directories('../../include')
endif

peci_dep = dependency('libpeci', required: true)

intelcpusensor_lib = static_library(
    'intelcpusensor',
    'IntelCPUSensorMain.cpp',
    'IntelCPUSensor.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep, peci_dep],
    include_directories: intel_cpu_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [intelcpusensor_lib],
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep, peci_dep],
    include_directories: intel_cpu_inc,
)
install_symlink(
    'intelcpusensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DINTELCPU']
