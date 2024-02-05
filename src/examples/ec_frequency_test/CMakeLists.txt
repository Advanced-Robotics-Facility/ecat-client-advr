project(ec_frequency_test)

# compile frequency test
add_executable(ec_frequency_test
               src/ec_frequency_test.cpp)
 
target_link_libraries(ec_frequency_test
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    ecat_master::tools_utils
    )

install(TARGETS ec_frequency_test
        DESTINATION bin
        COMPONENT applications)
