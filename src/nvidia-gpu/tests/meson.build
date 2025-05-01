gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: true)
    gtest_dep = declare_dependency(
        dependencies: [
            dependency('threads'),
            gtest_proj.dependency('gtest'),
            gtest_proj.dependency('gtest_main'),
        ],
    )
    gmock_dep = gtest_proj.dependency('gmock')
endif

gpusensor_test_include_dirs = [gpusensor_include_dir]

test(
    'nvidiagpusensor_test',
    executable(
        'nvidiagpusensor_test',
        'NvidiaGpuSensorTest.cpp',
        '../OcpMctpVdm.cpp',
        '../NvidiaGpuMctpVdm.cpp',
        implicit_include_directories: false,
        include_directories: gpusensor_test_include_dirs,
        dependencies: [gtest_dep, gmock_dep],
    ),
    workdir: meson.current_source_dir(),
)

test(
    'instancedb_test',
    executable(
        'instancedb_test',
        'InstanceDbTest.cpp',
        dependencies: [gtest_dep, gmock_dep],
        implicit_include_directories: false,
        include_directories: gpusensor_test_include_dirs,
    ),
    workdir: meson.current_source_dir(),
)
