if not get_option('nvme').allowed()
    subdir_done()
endif

nvme_srcs = files('NVMeSensor.cpp', 'NVMeSensorMain.cpp')
nvme_srcs += files('NVMeBasicContext.cpp')

nvme_deps = [default_deps, i2c, thresholds_dep, utils_dep, threads]

nvmesensor_lib = static_library(
    'nvmesensor',
    sources: nvme_srcs,
    dependencies: nvme_deps,
    include_directories: src_inc,
)
dbus_sensors_deps += declare_dependency(
    link_with: [nvmesensor_lib],
    dependencies: nvme_deps,
    include_directories: src_inc,
)
install_symlink(
    'nvmesensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DNVME']

