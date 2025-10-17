gpusensor_sources = files(
    'Inventory.cpp',
    'MctpRequester.cpp',
    'NvidiaDeviceDiscovery.cpp',
    'NvidiaEthPort.cpp',
    'NvidiaGpuDevice.cpp',
    'NvidiaGpuEnergySensor.cpp',
    'NvidiaGpuMctpVdm.cpp',
    'NvidiaGpuPowerPeakReading.cpp',
    'NvidiaGpuPowerSensor.cpp',
    'NvidiaGpuSensor.cpp',
    'NvidiaGpuVoltageSensor.cpp',
    'NvidiaPcieDevice.cpp',
    'NvidiaPcieInterface.cpp',
    'NvidiaPciePort.cpp',
    'NvidiaPciePortMetrics.cpp',
    'NvidiaSmaDevice.cpp',
    'OcpMctpVdm.cpp',
)

gpusensor_include_dir = include_directories('.', is_system: true)
sensor_include_dir = include_directories('../..')

executable(
    'nvidiagpusensor',
    gpusensor_sources + files('NvidiaGpuSensorMain.cpp'),
    implicit_include_directories: false,
    include_directories: [gpusensor_include_dir, sensor_include_dir],
    dependencies: [thresholds_dep, utils_dep],
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

