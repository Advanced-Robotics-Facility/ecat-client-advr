cmake_minimum_required(VERSION 3.0)


project(ecat-client-advr)

# c++17 support
set(CMAKE_CXX_STANDARD 17)

option(COMPILE_EXAMPLE "Enable to compile an example for the UDP communication" ON)
option(COMPILE_GUI "Enable to compile the client GUI" ON)
#option(COMPILE_GRAVITY_COMPENSATION "Enable to compile an example of the gravity compensation" OFF)

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

find_package(ecat_master REQUIRED)

if(${COMPILE_EXAMPLE})
	# compile example
	add_subdirectory(src/examples)
endif()


if(${COMPILE_GUI})
	  # compile ec gui
	  add_subdirectory(src/gui/ec_gui)
endif()
