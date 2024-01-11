project(ec_rt_code)


# compile main
add_executable(ec_rt_code
               src/ec_rt_code.cpp
               src/ec_rt_trajectory.cpp)
               
target_link_libraries(ec_rt_code
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    ecat_master::tools_utils
    )

install(TARGETS ec_rt_code
    DESTINATION bin)
