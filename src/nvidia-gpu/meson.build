gpusensor_sources = files(
    'Inventory.cpp',
    'MctpRequester.cpp',
    'NvidiaDeviceDiscovery.cpp',
    'NvidiaDriverInformation.cpp',
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

if not get_option('nvidia-gpu').allowed()
    subdir_done()
endif

gpusensor_include_dir = include_directories('.', is_system: true)
sensor_include_dir = include_directories('../..')

nvidiagpusensor_lib = static_library(
    'nvidiagpusensor',
    gpusensor_sources + files('NvidiaGpuSensorMain.cpp'),
    implicit_include_directories: false,
    include_directories: [gpusensor_include_dir, sensor_include_dir],
    dependencies: [thresholds_dep, utils_dep, mctpasio_dep],
)
dbus_sensors_deps += declare_dependency(
    link_with: [nvidiagpusensor_lib],
    dependencies: [thresholds_dep, utils_dep, mctpasio_dep],
    include_directories: [gpusensor_include_dir, sensor_include_dir],
)
install_symlink(
    'nvidiagpusensor',
    install_dir: get_option('libexecdir') / 'dbus-sensors',
    pointing_to: get_option('prefix') / get_option('bindir') / 'dbussensors',
)
dbus_sensors_cpp_args += ['-DNVIDIAGPU']
