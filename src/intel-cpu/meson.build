if get_option('intel-cpu').allowed()

    peci_incdirs = []
    if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
        peci_incdirs = ['../../include']
    endif

    peci_dep = dependency('libpeci', required: true)
    intel_cpu_inc = src_inc

    executable(
        'intelcpusensor',
        'IntelCPUSensorMain.cpp',
        'IntelCPUSensor.cpp',
        dependencies: [
            default_deps,
            gpiodcxx,
            thresholds_dep,
            utils_dep,
            peci_dep,
        ],
        include_directories: intel_cpu_inc,
        install: true,
        install_dir: '/usr/libexec/dbus-sensors',
    )
endif
