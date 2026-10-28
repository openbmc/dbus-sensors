amd_bulk_telemetry_dir = '../../amd-bulk-telemetry'

test(
    'AmdBulkTelemetry',
    executable(
        'test_AmdBulkTelemetry',
        'test_BulkTelemetryCommands.cpp',
        'test_BulkTelemetryMessage.cpp',
        'test_NbioCounters.cpp',
        'test_OsfpStats.cpp',
        'test_PcieState.cpp',
        'test_RdmaStats.cpp',
        'test_XrmacStats.cpp',
        files(
            amd_bulk_telemetry_dir / 'BulkTelemetryCommands.cpp',
            amd_bulk_telemetry_dir / 'BulkTelemetryMessage.cpp',
            amd_bulk_telemetry_dir / 'NbioCounters.cpp',
            amd_bulk_telemetry_dir / 'NetworkAdapter.cpp',
            amd_bulk_telemetry_dir / 'OcpVdmIana.cpp',
            amd_bulk_telemetry_dir / 'OsfpStats.cpp',
            amd_bulk_telemetry_dir / 'PcieState.cpp',
            amd_bulk_telemetry_dir / 'RdmaStats.cpp',
            amd_bulk_telemetry_dir / 'XrmacStats.cpp',
        ),
        dependencies: [ut_deps_list, utils_dep],
        implicit_include_directories: false,
        include_directories: include_directories(amd_bulk_telemetry_dir),
    ),
)
