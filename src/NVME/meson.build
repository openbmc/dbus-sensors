nvme_srcs = files('NVMeSensor.cpp', 'NVMeSensorMain.cpp')
nvme_srcs += files('NVMeBasicContext.cpp')

nvme_deps = [default_deps, i2c, thresholds_dep, utils_dep, threads]

executable(
    'nvmesensor',
    sources: nvme_srcs,
    dependencies: nvme_deps,
    include_directories: ['.', '../'],
    install: true,
)