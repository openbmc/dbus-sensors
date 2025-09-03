src_inc = include_directories('..')

executable(
    'psusensor',
    'PSUEvent.cpp',
    'PSUSensor.cpp',
    'PSUSensorMain.cpp',
    dependencies: [
        default_deps,
        devicemgmt_dep,
        pwmsensor_dep,
        thresholds_dep,
        utils_dep,
    ],
    include_directories: src_inc,
    install: get_option('psu').allowed(),
    install_dir: '/usr/libexec/dbus-sensors',
)
