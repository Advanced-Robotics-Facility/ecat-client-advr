project(ec_hydraulic)

# compile hydraulic
add_executable(${PROJECT_NAME}
               src/ec_hydraulic.cpp)
     
if ( Xenomai_FOUND )
    xenomai_target_bootstrap(${PROJECT_NAME} SKINS posix)
endif()
     
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ec_library
    ecat_master::tools_utils
    )

install(TARGETS ${PROJECT_NAME}
        DESTINATION bin
        COMPONENT applications)
