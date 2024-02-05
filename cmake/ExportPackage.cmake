set(PROJECT_NAME "ecat_client_advr")
set(PROJECT_VERSION "0.0.1")
set(PROJECT_CONF_IN ${CMAKE_CURRENT_LIST_DIR})

function(export_${PROJECT_NAME})

    set(CMAKE_INSTALL_DIR lib/cmake/${PROJECT_NAME})

    # write the package version file
    include(CMakePackageConfigHelpers)
    set(config_version_file ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake)
    write_basic_package_version_file(
        ${config_version_file}
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        )
        
    #create export for build tree
    export(EXPORT ${PROJECT_NAME}Targets
       NAMESPACE ${PROJECT_NAME}::
       FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
    )

    # Configure 'xxxConfig.cmake' for a build tree
    set(build_config ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
    configure_package_config_file(${PROJECT_CONF_IN}/${PROJECT_NAME}Config.cmake.in
        ${build_config}
        INSTALL_DESTINATION "${PROJECT_BINARY_DIR}"
        )

    install(
        EXPORT ${PROJECT_NAME}Targets
        NAMESPACE ${PROJECT_NAME}::
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${CMAKE_INSTALL_DIR}
        COMPONENT targets
        )

    set(install_config ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
    configure_package_config_file(
        ${PROJECT_CONF_IN}/${PROJECT_NAME}Config.cmake.in
        ${install_config}
        INSTALL_DESTINATION ${CMAKE_INSTALL_DIR}
        )

    # Install config files
    install(
        FILES
        ${config_version_file}
        ${install_config}
        DESTINATION ${CMAKE_INSTALL_DIR}
        COMPONENT configs
        )

endfunction()

