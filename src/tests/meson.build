gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: false)
    if gtest_proj.found()
        gtest_dep = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ],
        )
        gmock_dep = gtest_proj.dependency('gmock')
    else
        assert(
            not get_option('tests').allowed(),
            'Googletest is required if tests are enabled',
        )
    endif
endif

ut_deps_list = [gtest_dep]

ut_deps_list += default_deps

test(
    'test_utils',
    executable(
        'test_utils',
        'test_Utils.cpp',
        '../Utils.cpp',
        dependencies: ut_deps_list,
        implicit_include_directories: false,
        include_directories: src_inc,
    ),
)

test(
    'test_ipmb',
    executable(
        'test_ipmb',
        '../ipmb/IpmbSensor.cpp',
        '../Utils.cpp',
        '../ipmb/IpmbSDRSensor.cpp',
        'test_IpmbSensor.cpp',
        dependencies: ut_deps_list,
        link_with: [utils_a, thresholds_a, devicemgmt_a],
        implicit_include_directories: false,
        include_directories: src_inc,
    ),
)

test(
    'MCTPReactor',
    executable(
        'test_MCTPReactor',
        'test_MCTPReactor.cpp',
        '../mctp/MCTPReactor.cpp',
        '../mctp/MCTPEndpoint.cpp',
        dependencies: [gmock_dep, ut_deps_list, utils_dep],
        implicit_include_directories: false,
        include_directories: '../mctp',
    ),
)

test(
    'MCTPEndpoint',
    executable(
        'test_MCTPEndpoint',
        'test_MCTPEndpoint.cpp',
        '../mctp/MCTPEndpoint.cpp',
        dependencies: [gmock_dep, ut_deps_list, utils_dep],
        implicit_include_directories: false,
        include_directories: '../mctp',
    ),
)

test(
    'NvidiaGPU',
    executable(
        'test_NvidiaGPU',
        'test_NvidiaGpuSensorTest.cpp',
        'test_NvidiaDeviceInventoryMctpVdm.cpp',
        gpusensor_sources,
        dependencies: [gmock_dep, ut_deps_list, utils_dep, mctpasio_dep],
        link_with: [thresholds_a],
        implicit_include_directories: false,
        include_directories: '../nvidia-gpu',
    ),
)
