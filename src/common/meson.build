thresholds_a = static_library(
    'thresholds_a',
    'Thresholds.cpp',
    include_directories: [config_incdir],
    dependencies: default_deps,
)

thresholds_dep = declare_dependency(
    link_with: [thresholds_a],
    dependencies: default_deps,
)

utils_a = static_library(
    'utils_a',
    ['FileHandle.cpp', 'SensorPaths.cpp', 'Utils.cpp'],
    include_directories: [config_incdir],
    dependencies: default_deps,
)

utils_dep = declare_dependency(link_with: [utils_a], dependencies: [sdbusplus])

devicemgmt_a = static_library(
    'devicemgmt_a',
    ['DeviceMgmt.cpp'],
    include_directories: [config_incdir],
    dependencies: default_deps,
)

devicemgmt_dep = declare_dependency(
    link_with: [devicemgmt_a],
    dependencies: default_deps,
)

pwmsensor_a = static_library(
    'pwmsensor_a',
    'PwmSensor.cpp',
    include_directories: [config_incdir],
    dependencies: [default_deps, thresholds_dep],
)

pwmsensor_dep = declare_dependency(
    link_with: [pwmsensor_a],
    dependencies: [default_deps, thresholds_dep],
)

systemdinterface_src = files('SystemdInterface.cpp')

gpiointerface_a = static_library(
    'gpiointerface_a',
    'GPIOInterface.cpp',
    include_directories: [config_incdir],
    dependencies: [default_deps, gpiodcxx],
)

gpiointerface_dep = declare_dependency(
    link_with: [gpiointerface_a],
    dependencies: default_deps,
)

entitymanagerinterface_a = static_library(
    'entitymanagerinterface_a',
    'EntityManagerInterface.cpp',
    include_directories: [config_incdir],
    dependencies: default_deps,
)

entitymanagerinterface_dep = declare_dependency(
    link_with: [entitymanagerinterface_a],
    dependencies: default_deps,
)

notifywatch_a = static_library(
    'notifywatch_a',
    'NotifyWatch.cpp',
    include_directories: [config_incdir],
    dependencies: default_deps,
)

notifywatch_dep = declare_dependency(
    link_with: [notifywatch_a],
    dependencies: default_deps,
)
