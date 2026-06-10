gpusensor_dir = '../../nvidia-gpu'

gpusensor_sources = files(
    gpusensor_dir / 'Inventory.cpp',
    gpusensor_dir / 'DeviceManager.cpp',
    gpusensor_dir / 'NvidiaDriverInformation.cpp',
    gpusensor_dir / 'NvidiaEthPort.cpp',
    gpusensor_dir / 'NvidiaEventReporting.cpp',
    gpusensor_dir / 'NvidiaGpuClockFrequencyMetric.cpp',
    gpusensor_dir / 'NvidiaGpuEccMode.cpp',
    gpusensor_dir / 'NvidiaGpuMemoryCapacityUtilization.cpp',
    gpusensor_dir / 'NvidiaGpuClockSpeedControl.cpp',
    gpusensor_dir / 'NvidiaGpuPowerControl.cpp',
    gpusensor_dir / 'NvidiaGpuUtilizationMetrics.cpp',
    gpusensor_dir / 'NvidiaGpuDevice.cpp',
    gpusensor_dir / 'NvidiaGpuEnergySensor.cpp',
    gpusensor_dir / 'NvidiaGpuLongRunningCommand.cpp',
    gpusensor_dir / 'NvidiaGpuMctpVdm.cpp',
    gpusensor_dir / 'NvidiaGpuMemoryClockFrequency.cpp',
    gpusensor_dir / 'NvidiaGpuMemoryDevice.cpp',
    gpusensor_dir / 'NvidiaGpuPowerPeakReading.cpp',
    gpusensor_dir / 'NvidiaGpuResetRequired.cpp',
    gpusensor_dir / 'NvidiaGpuPowerSensor.cpp',
    gpusensor_dir / 'NvidiaGpuTempSensor.cpp',
    gpusensor_dir / 'NvidiaGpuViolationDuration.cpp',
    gpusensor_dir / 'NvidiaGpuVoltageSensor.cpp',
    gpusensor_dir / 'NvidiaGpuXid.cpp',
    gpusensor_dir / 'NvidiaLongRunningHandler.cpp',
    gpusensor_dir / 'NvidiaPcieDevice.cpp',
    gpusensor_dir / 'NvidiaPcieFunction.cpp',
    gpusensor_dir / 'NvidiaPcieInterface.cpp',
    gpusensor_dir / 'NvidiaPciePort.cpp',
    gpusensor_dir / 'NvidiaPciePortMetrics.cpp',
    gpusensor_dir / 'NvidiaSmaDevice.cpp',
    gpusensor_dir / 'OcpMctpVdm.cpp',
    gpusensor_dir / 'SerialQueue.cpp',
)

dbus_fixture_sources = files(
    'utils/DbusMockTestBase.cpp',
    'utils/LoggingStub.cpp',
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
        'test_NvidiaGpuDevice.cpp',
        'test_NvidiaGpuPowerControl.cpp',
        'test_NvidiaGpuClockFrequencyMetric.cpp',
        'test_NvidiaGpuMemoryClockFrequency.cpp',
        'test_NvidiaGpuMemoryDevice.cpp',
        'test_NvidiaPcieFunction.cpp',
        'test_NvidiaGpuUtilizationMetrics.cpp',
        'test_NvidiaGpuViolationDuration.cpp',
        'test_SerialQueue.cpp',
        'test_NvidiaLongRunningHandler.cpp',
        'test_NvidiaEventReporting.cpp',
        'test_NvidiaGpuXid.cpp',
        'test_Inventory.cpp',
        'test_NvidiaSmaDevice.cpp',
        'test_NvidiaGpuClockSpeedControl.cpp',
        'test_NvidiaGpuResetRequired.cpp',
        gpusensor_sources,
        dbus_fixture_sources,
        mock_mctp_sources,
        dependencies: [gmock_dep, ut_deps_list, utils_dep, mctpasio_dep],
        link_with: [thresholds_a],
        implicit_include_directories: false,
        include_directories: gpusensor_inc,
    ),
)

test(
    'EndpointState',
    executable(
        'test_EndpointState',
        'test_EndpointState.cpp',
        dependencies: [gtest_dep, gmock_dep],
        implicit_include_directories: false,
        include_directories: gpusensor_dir,
    ),
)
