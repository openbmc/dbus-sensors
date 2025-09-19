systemdinterface_src = files('SystemdInterface.cpp')

gpiointerface_a = static_library(
    'gpiointerface_a',
    'GPIOInterface.cpp',
    include_directories: [config_incdir, src_inc],
    dependencies: [default_deps, gpiodcxx],
)

gpiointerface_dep = declare_dependency(
    link_with: [gpiointerface_a],
    dependencies: default_deps,
)

entitymanagerinterface_a = static_library(
    'entitymanagerinterface_a',
    'EntityManagerInterface.cpp',
    include_directories: [config_incdir, src_inc],
    dependencies: default_deps,
)

entitymanagerinterface_dep = declare_dependency(
    link_with: [entitymanagerinterface_a],
    dependencies: default_deps,
)

notifywatch_a = static_library(
    'notifywatch_a',
    'NotifyWatch.cpp',
    include_directories: [config_incdir, src_inc],
    dependencies: default_deps,
)

notifywatch_dep = declare_dependency(
    link_with: [notifywatch_a],
    dependencies: default_deps,
)
