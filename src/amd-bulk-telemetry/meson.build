amd_bulk_telemetry_sources = files('AmdBulkSensorMain.cpp')

if not get_option('amd-bulk-telemetry').allowed()
    subdir_done()
endif

amd_bulk_telemetry_include_dir = include_directories('.', is_system: true)

executable(
    'amdbulksensor',
    amd_bulk_telemetry_sources,
    implicit_include_directories: false,
    include_directories: [amd_bulk_telemetry_include_dir],
    dependencies: [utils_dep],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
