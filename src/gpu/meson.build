gpusensor_sources = files('GpuSensor.cpp', 'GpuSensorMain.cpp')

gpusensor_include_dir = include_directories('.', is_system: true)
sensor_include_dir = include_directories('../..')

executable(
    'gpusensor',
    gpusensor_sources,
    implicit_include_directories: false,
    include_directories: [gpusensor_include_dir, sensor_include_dir],
    dependencies: [thresholds_dep, utils_dep],
    install: true,
)
