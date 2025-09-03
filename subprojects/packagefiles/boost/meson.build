project('boost', 'cpp', version: '1.89.0', license: 'Boost')

cmake = import('cmake')
opt = cmake.subproject_options()
boost_libs = ['asio', 'headers']
opt.add_cmake_defines(
    {
        'BOOST_INCLUDE_LIBRARIES': ';'.join(boost_libs),
        'BUILD_SHARED_LIBS': 'OFF',
    },
)

boost_cmake = cmake.subproject('boost', required: true, options: opt)
boost_deps = []
foreach boost_lib : boost_libs
    boost_lib_instance = boost_cmake.dependency('boost_' + boost_lib).as_system()
    boost_deps += [boost_lib_instance]
endforeach
boost_dep = declare_dependency(dependencies: boost_deps)

meson.override_dependency('boost', boost_dep)
