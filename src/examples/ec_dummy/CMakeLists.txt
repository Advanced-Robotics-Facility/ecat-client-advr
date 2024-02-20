project(ec_dummy)

# compile frequency test
add_executable(${PROJECT_NAME}
               src/ec_dummy.cpp)
               
if ( Xenomai_FOUND )
    xenomai_target_bootstrap(${PROJECT_NAME} SKINS posix)
endif()
 
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_library
    ecat_master::tools_utils
    )
    
install(TARGETS ${PROJECT_NAME}
        DESTINATION bin
        COMPONENT applications)
