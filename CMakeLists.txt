project(ecat-client-advr)
cmake_minimum_required(VERSION 3.0)

# std c++17 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)
set(ENABLE_XENO OFF CACHE BOOL "Enable xenomai real-time features")
set(ENABLE_PREEMPT_RT OFF CACHE BOOL "Enable preempt-rt real-time features")
option(COMPILE_EXAMPLE "Enable to compile examples" ON)
option(COMPILE_GUI "Enable to compile the GUI" OFF)
option(COMPILE_MATLAB "Enable to compile the matlab" OFF)


add_compile_options(-Wall -Wextra -Werror)
#add_compile_options(-Wno-unused)
add_compile_options(-Wno-unused-but-set-variable
                    -Wno-unused-variable
                    -Wno-unused-parameter
                    -Wno-unused-function)
# suppress warning: ‘int google::protobuf::MessageLite::ByteSize() const’ is deprecated
# Please use ByteSizeLong() instead [-Wdeprecated-declarations]
add_compile_options(-Wno-deprecated-declarations)

find_package(Boost REQUIRED system)

find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Git REQUIRED)
execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${CMAKE_SOURCE_DIR}
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

set(EXTERNAL "${PROJECT_SOURCE_DIR}/external")

if( ENABLE_XENO AND ENABLE_PREEMPT_RT)
    message( FATAL_ERROR "Cannot setup both Xenomai and PREEMPT_RT Real-Time OS!")
endif()

if ( ${ENABLE_XENO} )
    option(CMAKE_USE_XENOMAI "Use Xenomai libraries" ON)
    find_package(Xenomai REQUIRED)
    if(TARGET Xenomai::cobalt)
        set(USE_COBALT True)
    endif()
else ()
    option(CMAKE_USE_XENOMAI "Use Xenomai libraries" OFF)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

if(ENABLE_PREEMPT_RT)
    ADD_DEFINITIONS(-DPREEMPT_RT)
endif(ENABLE_PREEMPT_RT)

find_package(magic_enum)
if(NOT ${magic_enum_FOUND})
    set(MAGIC_ENUM_OPT_INSTALL ON CACHE BOOL "Enable option to install Magic Enum" FORCE)
    include(cmake/CPM.cmake)
    CPMAddPackage(
        NAME magic_enum
        GITHUB_REPOSITORY Neargye/magic_enum
        GIT_TAG v0.8.1 # Where `x.y.z` is the release version you want to use.
    )
endif()

find_package(matlogger2 REQUIRED)

find_package(ecat_master REQUIRED COMPONENTS ecat_udp ecat_core)

# set includes
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    )
    
#compile ec_library
add_subdirectory(src/library/ec_library/)
add_subdirectory(src/simulator/repl_dummy)

if(${COMPILE_EXAMPLE})
    # compile example
    add_subdirectory(src/examples)
endif()


if(${COMPILE_GUI})
    # compile ec gui
    add_subdirectory(src/gui/ec_gui)
endif()

if(${COMPILE_MATLAB})
    # compile ec block
    add_subdirectory(src/matlab/ec_block/)
endif()

install(
       DIRECTORY config
       DESTINATION share/${PROJECT_NAME}
       COMPONENT configs
       FILES_MATCHING 
       PATTERN "*.*")

# Debian package for the ADVR Facility Debian package
if (NOT CPACK_SYSTEM_NAME)
    set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_PROCESSOR}")
    if (CPACK_SYSTEM_NAME STREQUAL "x86_64")
        set(CPACK_SYSTEM_NAME "amd64")
    endif ()
endif ()

set(VERSION_LIB "0.0.1")
set(REVISION "0.0.1")

set(DEB_INSTALL_PREFIX "/usr/local" CACHE STRING "Debs install prefix")

include (InstallRequiredSystemLibraries)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_PACKAGING_INSTALL_PREFIX ${DEB_INSTALL_PREFIX})
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}_package)
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}${LIB_TYPE}_package-${VERSION_LIB}-r${REVISION}-${CPACK_SYSTEM_NAME}")
# note: comment out next line to be able to install ros msg and srv
#set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};applications;/;${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};libraries;/;${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};headers;/;${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};configs;/;${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};targets;/")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS OFF)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "davide.antonucci@iit.it")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "ADVR Facility EtherCAT Client")
include(CPack)

include(cmake/ExportPackage.cmake)
export_ecat_client_advr()


