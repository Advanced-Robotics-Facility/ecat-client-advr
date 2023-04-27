project(ec_frequency_test)


OPTION(TEST_MAIN "Testing Main concepts" OFF)

IF(TEST_MAIN)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_MAIN)

    
# compile frequency test
add_executable(ec_frequency_test
               src/ec_frequency_test.cpp)
 
target_link_libraries(ec_frequency_test
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    )


install(TARGETS ec_frequency_test
    DESTINATION bin)
