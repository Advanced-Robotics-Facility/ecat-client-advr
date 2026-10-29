
OPTION(TEST_EXAMPLES "Testing Examples" OFF)
OPTION(COMPILE_GRAVITY_COMPENSATION "Compile gravity compensation" OFF)
OPTION(COMPILE_ROS2_CLIENT "Compile ros2 client" OFF)

IF(TEST_EXAMPLES)
    ADD_DEFINITIONS(-DTEST_EXAMPLES)
ENDIF(TEST_EXAMPLES)

add_subdirectory(ec_simple_trajectory)
IF(COMPILE_GRAVITY_COMPENSATION)
add_subdirectory(ec_gravity_compensation)
ENDIF(COMPILE_GRAVITY_COMPENSATION)
add_subdirectory(ec_frequency_test)
add_subdirectory(ec_hydraulic)
add_subdirectory(ec_general_trajectory)
IF(COMPILE_ROS2_CLIENT)
add_subdirectory(ec_ros2)
ENDIF(COMPILE_ROS2_CLIENT)
