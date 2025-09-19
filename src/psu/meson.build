if not get_option('psu').allowed()
    subdir_done()
endif

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
    include_directories: [src_inc, config_incdir],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

