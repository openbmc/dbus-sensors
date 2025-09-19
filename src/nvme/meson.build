nvme_srcs = files('NVMeSensor.cpp', 'NVMeSensorMain.cpp')
nvme_srcs += files('NVMeBasicContext.cpp')

nvme_deps = [default_deps, i2c, thresholds_dep, utils_dep, threads]
src_inc = include_directories('..')

executable(
    'nvmesensor',
    sources: nvme_srcs,
    dependencies: nvme_deps,
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
