project(ec_gravity_compensation)

find_package(XBotInterface REQUIRED)

# compile gravity_compensation
add_executable(ec_gravity_compensation
               src/ec_gravity_compensation.cpp)
               
target_link_libraries(ec_gravity_compensation
    PRIVATE 
    ${XBotInterface_LIBRARIES}
    PUBLIC
    ec_library
    ecat_master::tools_utils
    )
    

install(TARGETS ec_gravity_compensation
        DESTINATION bin
        COMPONENT applications)
