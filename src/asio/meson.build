mctpasio_a = static_library(
    'mctpasio_a',
    'MctpAsioEndpoint.cpp',
    dependencies: [boost, phosphor_logging_dep],
)

mctpasio_dep = declare_dependency(
    include_directories: ['..'],
    link_with: [mctpasio_a],
    dependencies: default_deps,
)

thresholds_a = static_library(
    'thresholds_a',
    'Thresholds.cpp',
    include_directories: [config_incdir, src_inc],
    dependencies: default_deps,
)

thresholds_dep = declare_dependency(
    link_with: [thresholds_a],
    dependencies: default_deps,
)

devicemgmt_a = static_library(
    'devicemgmt_a',
    ['DeviceMgmt.cpp'],
    include_directories: [config_incdir, src_inc],
    dependencies: default_deps,
)

devicemgmt_dep = declare_dependency(
    link_with: [devicemgmt_a],
    dependencies: default_deps,
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'PwmSensor.cpp',
    include_directories: [config_incdir, src_inc],
    dependencies: [default_deps, thresholds_dep],
)

pwmsensor_dep = declare_dependency(
    link_with: [pwmsensor_a],
    dependencies: [default_deps, thresholds_dep],
)
