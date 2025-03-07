gpusensor_sources = files('GpuSensorMain.cpp')

libgpusensor_sources = files(
    'GpuMctpVdm.cpp',
    'GpuSensor.cpp',
    'MctpRequester.cpp',
)

libgpusensor_include_dir = include_directories('.', is_system: true)
sensor_include_dir = include_directories('../..')

libgpusensor = static_library(
    'libgpusensor',
    libgpusensor_sources,
    dependencies: [default_deps, thresholds_dep, utils_dep],
    implicit_include_directories: false,
    include_directories: [libgpusensor_include_dir, sensor_include_dir],
    gnu_symbol_visibility: 'default',
    install: true,
)

libgpusensor_dep = declare_dependency(
    include_directories: [libgpusensor_include_dir, sensor_include_dir],
    link_with: libgpusensor,
)

executable(
    'gpusensor',
    gpusensor_sources,
    dependencies: [libgpusensor_dep, thresholds_dep, utils_dep],
    install: true,
)

if get_option('tests').enabled()
    subdir('tests')
endif
