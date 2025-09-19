utils_a = static_library(
    'utils_a',
    ['FileHandle.cpp', 'SensorPaths.cpp', 'Utils.cpp'],
    include_directories: [config_incdir, src_inc],
    dependencies: default_deps,
)

utils_dep = declare_dependency(link_with: [utils_a], dependencies: [sdbusplus])
