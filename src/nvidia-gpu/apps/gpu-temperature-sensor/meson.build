gpusensor_sources = files(
        'GpuMain.cpp',
)

libgpusensor_sources = files(
        'GpuSensorTask.cpp',
        'GpuTempSensor.cpp',
        'MctpRequester.cpp',
)

libgpusensor_include_dir = include_directories('.', is_system: true)

libgpusensor = static_library(
    'libgpusensor',
    libgpusensor_sources,
    dependencies: [
        libocpami_dep,
        default_deps,
    ],
    implicit_include_directories: false,
    include_directories: [libgpusensor_include_dir],
    gnu_symbol_visibility: 'default',
    install: true,
)

libgpusensor_dep = declare_dependency(
    include_directories: libgpusensor_include_dir,
    link_with: libgpusensor,
)

executable(
    'gpusensor',
    gpusensor_sources,
    dependencies: [
        libgpusensor_dep,
        libocpami_dep,
        default_deps,
    ],
    install: true
)
