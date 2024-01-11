
OPTION(TEST_EXAMPLES "Testing Examples" OFF)

IF(TEST_EXAMPLES)
    ADD_DEFINITIONS(-DTEST_EXAMPLES)
ENDIF(TEST_EXAMPLES)

add_subdirectory(ec_simple_trajectory)
add_subdirectory(ec_gravity_compensation)
add_subdirectory(ec_frequency_test)
add_subdirectory(ec_rt_code)

