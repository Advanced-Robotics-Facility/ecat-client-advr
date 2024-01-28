
OPTION(TEST_EXAMPLES "Testing Examples" OFF)
OPTION(COMPILE_GRAVITY_COMPENSATION "Compile gravity compensation" OFF)

IF(TEST_EXAMPLES)
    ADD_DEFINITIONS(-DTEST_EXAMPLES)
ENDIF(TEST_EXAMPLES)

add_subdirectory(ec_simple_trajectory)
IF(COMPILE_GRAVITY_COMPENSATION)
add_subdirectory(ec_gravity_compensation)
ENDIF(COMPILE_GRAVITY_COMPENSATION)
add_subdirectory(ec_frequency_test)

