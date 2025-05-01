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

mctp_test_include_dirs = [
    include_directories('..'),
    include_directories('../..'),
]

test(
    'requester_test',
    executable(
        'requester_test',
        'RequesterTest.cpp',
        implicit_include_directories: false,
        include_directories: mctp_test_include_dirs,
        dependencies: [gtest_dep, gmock_dep, boost, phosphor_logging_dep, uring],
    ),
)
