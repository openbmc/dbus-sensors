gpusensor_dir = '../../nvidia-gpu'

gpusensor_sources = files(
    gpusensor_dir / 'NvidiaGpuMctpVdm.cpp',
    gpusensor_dir / 'OcpMctpVdm.cpp',
    gpusensor_dir / 'NvidiaGpuSensor.cpp',
)

mock_mctp_sources = files(
    'utils/MockDbusHelper.cpp',
    'utils/MockMctpRequester.cpp',
)

gpusensor_inc = include_directories(
    'utils',
    gpusensor_dir,
)

test(
    'NvidiaGpuSensors',
    executable(
        'test_NvidiaGpuSensors',
        'test_NvidiaGpuMctpVdm.cpp',
        'test_NvidiaDeviceInventoryMctpVdm.cpp',
        'test_NvidiaGpuSensor.cpp',
        gpusensor_sources,
        mock_mctp_sources,
        dependencies: [gmock_dep, ut_deps_list, utils_dep, mctpasio_dep],
        link_with: [thresholds_a],
        implicit_include_directories: false,
        include_directories: gpusensor_inc,
    ),
)
