gpusensor_dir = '../../nvidia-gpu'

gpusensor_sources = files(
    gpusensor_dir / 'NvidiaGpuMctpVdm.cpp',
    gpusensor_dir / 'OcpMctpVdm.cpp',
)

gpusensor_inc = include_directories(
    gpusensor_dir,
)

test(
    'NvidiaGpuSensors',
    executable(
        'test_NvidiaGpuSensors',
        'test_NvidiaGpuMctpVdm.cpp',
        'test_NvidiaDeviceInventoryMctpVdm.cpp',
        gpusensor_sources,
        dependencies: [gmock_dep, ut_deps_list, utils_dep, mctpasio_dep],
        link_with: [thresholds_a],
        implicit_include_directories: false,
        include_directories: gpusensor_inc,
    ),
)
