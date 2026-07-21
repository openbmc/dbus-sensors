gpusensor_dir = '../../nvidia-gpu'

gpusensor_sources = files(
    gpusensor_dir / 'Inventory.cpp',
    gpusensor_dir / 'NvidiaDriverInformation.cpp',
    gpusensor_dir / 'NvidiaEthPort.cpp',
    gpusensor_dir / 'NvidiaGpuEnergySensor.cpp',
    gpusensor_dir / 'NvidiaGpuMctpVdm.cpp',
    gpusensor_dir / 'NvidiaGpuPowerPeakReading.cpp',
    gpusensor_dir / 'OcpMctpVdm.cpp',
    gpusensor_dir / 'NvidiaGpuPowerSensor.cpp',
    gpusensor_dir / 'NvidiaGpuTempSensor.cpp',
    gpusensor_dir / 'NvidiaGpuVoltageSensor.cpp',
    gpusensor_dir / 'NvidiaPcieDevice.cpp',
    gpusensor_dir / 'NvidiaPcieFunction.cpp',
    gpusensor_dir / 'NvidiaPcieInterface.cpp',
    gpusensor_dir / 'NvidiaPciePort.cpp',
    gpusensor_dir / 'NvidiaPciePortMetrics.cpp',
)

dbus_fixture_sources = files(
    'utils/DbusMockTestBase.cpp',
)

mock_mctp_sources = files(
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
        'test_NvidiaGpuTempSensor.cpp',
        'test_NvidiaGpuPowerSensor.cpp',
        'test_NvidiaGpuEnergySensor.cpp',
        'test_NvidiaGpuVoltageSensor.cpp',
        'test_NvidiaGpuPowerPeakReading.cpp',
        'test_NvidiaDriverInformation.cpp',
        'test_NvidiaPcieInterface.cpp',
        'test_NvidiaPciePort.cpp',
        'test_NvidiaPciePortMetrics.cpp',
        'test_NvidiaEthPort.cpp',
        'test_NvidiaPcieDevice.cpp',
        'test_NvidiaPcieFunction.cpp',
        'test_Inventory.cpp',
        gpusensor_sources,
        dbus_fixture_sources,
        mock_mctp_sources,
        dependencies: [gmock_dep, ut_deps_list, utils_dep, mctpasio_dep],
        link_with: [thresholds_a],
        implicit_include_directories: false,
        include_directories: gpusensor_inc,
    ),
)
