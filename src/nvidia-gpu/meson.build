gpusensor_sources = files(
    'MctpRequester.cpp',
    'NvidiaDeviceDiscovery.cpp',
    'NvidiaGpuDevice.cpp',
    'NvidiaGpuMctpVdm.cpp',
    'NvidiaGpuSensor.cpp',
    'NvidiaGpuSensorMain.cpp',
    'OcpMctpVdm.cpp',
    'Inventory.cpp',
    'Memory.cpp',
)

gpusensor_include_dir = include_directories('.', is_system: true)
sensor_include_dir = include_directories('../..')

executable(
    'nvidiagpusensor',
    gpusensor_sources,
    implicit_include_directories: false,
    include_directories: [gpusensor_include_dir, sensor_include_dir],
    dependencies: [thresholds_dep, utils_dep],
    install: true,
)

if get_option('tests').enabled()
    subdir('tests')
endif
