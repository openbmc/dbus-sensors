gpusensor_sources = files(
    'MctpRequester.cpp',
    'NvidiaGpuMctpVdm.cpp',
    'NvidiaGpuSensor.cpp',
    'NvidiaGpuSensorMain.cpp',
    'OcpMctpVdm.cpp',
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
    install_dir: '/usr/libexec/dbus-sensors',
)

if get_option('tests').enabled()
    subdir('tests')
endif
