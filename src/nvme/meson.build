if get_option('nvme').allowed()

    nvme_srcs = files('NVMeSensor.cpp', 'NVMeSensorMain.cpp')
    nvme_srcs += files('NVMeBasicContext.cpp')

    nvme_deps = [default_deps, i2c, thresholds_dep, utils_dep, threads]

    executable(
        'nvmesensor',
        sources: nvme_srcs,
        dependencies: nvme_deps,
        include_directories: src_inc,
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
