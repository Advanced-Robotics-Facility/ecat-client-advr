project(ec_simple_trajectory)


# compile main
add_executable(ec_simple_trajectory
               src/ec_simple_trajectory.cpp)
               
target_link_libraries(ec_simple_trajectory
    PUBLIC
    ec_library
    ecat_master::tools_utils
    )

install(TARGETS ec_simple_trajectory
        DESTINATION bin
        COMPONENT applications)
