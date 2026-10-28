amd_bulk_telemetry_sources = files(
    'AmdBulkDevice.cpp',
    'BulkTelemetryCommands.cpp',
    'BulkTelemetryMessage.cpp',
    'MctpDiscovery.cpp',
    'NetworkAdapter.cpp',
    'OcpVdmIana.cpp',
    'OsfpStats.cpp',
)

if not get_option('amd-bulk-telemetry').allowed()
    subdir_done()
endif

amd_bulk_telemetry_include_dir = include_directories('.', is_system: true)

phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')

executable(
    'amdbulksensor',
    amd_bulk_telemetry_sources + files('AmdBulkSensorMain.cpp'),
    implicit_include_directories: false,
    include_directories: [amd_bulk_telemetry_include_dir],
    dependencies: [
        entitymanagerinterface_dep,
        mctpasync_dep,
        phosphor_dbus_interfaces_dep,
        phosphor_logging_dep,
        utils_dep,
    ],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
