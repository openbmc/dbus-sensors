peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../../include']
endif

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
    include_directories: ['.', '../', peci_incdirs],
    install: true,
)