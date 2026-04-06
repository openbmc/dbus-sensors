mock_mctp_sources = files(
    'utils/MockDbusHelper.cpp',
    'utils/MockMctpRequester.cpp',
)

gpusensor_sources = files(
    '../../nvidia-gpu/NvidiaGpuMctpVdm.cpp',
    '../../nvidia-gpu/NvidiaGpuSensor.cpp',
    '../../nvidia-gpu/OcpMctpVdm.cpp',
)

nvidia_gpu_test_inc = include_directories(
    'utils',
    '../../nvidia-gpu',
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
        include_directories: nvidia_gpu_test_inc,
    ),
)
