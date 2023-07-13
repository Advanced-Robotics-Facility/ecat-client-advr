project(ec_simple_trajectory)


# compile main
add_executable(ec_simple_trajectory
               src/ec_simple_trajectory.cpp)
               
target_link_libraries(ec_simple_trajectory
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    )

install(TARGETS ec_simple_trajectory
    DESTINATION bin)
