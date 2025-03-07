add_languages('c', required: true)

libocpami_sources = files(
    'af-mctp.c',
    'nvidia_common.c',
    'nvidia_sensors.c',
    'ocp_ami.c',
    'transport.c',
)

libocpami_headers = files(
    'af-mctp.h',
    'nvidia_common.h',
    'nvidia_sensors.h',
    'ocp_ami.h',
    'transport.h',
)

libocpami_include_dir = include_directories('.', is_system: true)

libocpami = library(
    'ocpami',
    libocpami_sources,
    implicit_include_directories: false,
    include_directories: [libocpami_include_dir],
    version: meson.project_version(),
    gnu_symbol_visibility: 'default',
    install: true,
)

libocpami_dep = declare_dependency(
    include_directories: libocpami_include_dir,
    link_with: libocpami,
)

install_headers(libocpami_headers, subdir: 'libocpami', preserve_path: true)

import('pkgconfig').generate(
    name: 'libocpami',
    description: 'OCP GPU & Accelerator Management Interfaces protocol encode/decode C lib',
    version: meson.project_version(),
    libraries: libocpami,
)

if get_option('tests').enabled()
    subdir('tests')
endif
