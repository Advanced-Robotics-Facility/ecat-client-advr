project(ec_gravity_compensation)

find_package(XBotInterface REQUIRED)

# compile gravity_compensation
add_executable(ec_gravity_compensation
               src/ec_gravity_compensation.cpp)
               
target_link_libraries(ec_gravity_compensation
    PRIVATE 
    ${XBotInterface_LIBRARIES}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    )
    

install(TARGETS ec_gravity_compensation
    DESTINATION bin)

