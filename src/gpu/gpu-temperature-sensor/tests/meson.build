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

libocpami_test_include_dirs = [libgpusensor_include_dir]

test(
    'gpusensor_test',
    executable(
<<<<<<< HEAD:src/gpu/gpu-temperature-sensor/tests/meson.build
        'gpusensor_test',
        'GpuSensorsTest.cpp',
=======
        'libocpami_test',
        'ocp_ami_test.cpp',
        'gpu_common_test.cpp',
        'gpu_sensors_test.cpp',
>>>>>>> a578aa7 (nvidia-gpu : add nvidia oem extension to libocpami):src/gpu/lib/tests/meson.build
        implicit_include_directories: false,
        include_directories: libocpami_test_include_dirs,
        dependencies: [libgpusensor_dep, gtest_dep, gmock_dep],
    ),
    workdir: meson.current_source_dir(),
)
