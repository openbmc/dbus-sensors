project('boost',
    'cpp',
    version : '1.83.0',
    license : 'Boost'
)

boost_dep = declare_dependency(
    include_directories : include_directories('.'),
)

meson.override_dependency('boost', boost_dep)
