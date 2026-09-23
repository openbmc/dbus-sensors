amd_bulk_telemetry_dir = '../../amd-bulk-telemetry'

test(
    'AmdBulkTelemetry',
    executable(
        'test_AmdBulkTelemetry',
        'test_BulkTelemetryCommands.cpp',
        'test_BulkTelemetryMessage.cpp',
        files(
            amd_bulk_telemetry_dir / 'BulkTelemetryCommands.cpp',
            amd_bulk_telemetry_dir / 'BulkTelemetryMessage.cpp',
            amd_bulk_telemetry_dir / 'OcpVdmIana.cpp',
        ),
        dependencies: [ut_deps_list, utils_dep],
        implicit_include_directories: false,
        include_directories: include_directories(amd_bulk_telemetry_dir),
    ),
)
