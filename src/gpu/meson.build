gpusensor_sources = files(
    'GpuDevice.cpp',
    'GpuMctpVdm.cpp',
    'GpuSensor.cpp',
    'GpuSensorMain.cpp',
    'GpuThresholds.cpp',
    'OcpMctpVdm.cpp',
)

gpusensor_include_dir = include_directories('.', is_system: true)
mctp_requester_include_dir = include_directories('mctp')
sensor_include_dir = include_directories('../..')

executable(
    'gpusensor',
    gpusensor_sources,
    implicit_include_directories: false,
    include_directories: [
        gpusensor_include_dir,
        sensor_include_dir,
        mctp_requester_include_dir,
    ],
    dependencies: [thresholds_dep, utils_dep],
    install: true,
)

if get_option('tests').enabled()
    subdir('tests')
    subdir('mctp/tests')
endif
