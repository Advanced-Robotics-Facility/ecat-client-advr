project(ec_simple_trajectory)

OPTION(TEST_MAIN "Testing Main concepts" OFF)

IF(TEST_MAIN)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_MAIN)

# compile main
add_executable(ec_simple_trajectory
               src/ec_simple_trajectory.cpp
               ${PROJECT_SOURCE_DIR}/../../utils/ec_utils.cpp)
               
target_link_libraries(ec_simple_trajectory
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    )

install(TARGETS ec_simple_trajectory
    DESTINATION bin)
