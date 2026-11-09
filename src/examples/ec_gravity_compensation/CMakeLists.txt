project(ec_gravity_compensation)

find_package(xbot2_interface REQUIRED)

# compile gravity_compensation
add_executable(${PROJECT_NAME}
               src/ec_gravity_compensation.cpp)
               
if ( Xenomai_FOUND )
    xenomai_target_bootstrap(${PROJECT_NAME} SKINS posix)
endif()
               
target_link_libraries(${PROJECT_NAME}
    PRIVATE 
    xbot2_interface::xbot2_interface
    PUBLIC
    ec_library
    ecat_master::tools_utils
    )
    

install(TARGETS ${PROJECT_NAME}
        DESTINATION bin
        COMPONENT applications)
