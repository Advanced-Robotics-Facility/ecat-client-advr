project(ecat-client-advr)
cmake_minimum_required(VERSION 3.0)

# std c++17 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

option(COMPILE_EXAMPLE "Enable to compile examples" ON)
option(COMPILE_GUI "Enable to compile the GUI" ON)
option(COMPILE_MATLAB "Enable to compile the matlab" ON)

find_package(Boost REQUIRED system)

find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Git REQUIRED)
execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${CMAKE_SOURCE_DIR}
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

set(EXTERNAL "${PROJECT_SOURCE_DIR}/external")

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
    ${PROJECT_SOURCE_DIR}/src/utils
    ${PROJECT_SOURCE_DIR}/src/library/protocols/udp
    ${EIGEN3_INCLUDE_DIR}
    )
    
#compile ec_library
add_subdirectory(src/library)

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
