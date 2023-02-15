cmake_minimum_required(VERSION 3.0)

project(ec_client_udp)

option(COMPILE_EXAMPLE "Enable to compile an example for the UDP communication" ON)
option(COMPILE_GUI "Enable to compile the client GUI" ON)
option(COMPILE_GRAVITY_COMPENSATION "Enable to compile an example of the gravity compensation" OFF)

find_package(Git REQUIRED)
execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${CMAKE_SOURCE_DIR}
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

include(cmake/CPM.cmake)

CPMAddPackage(
    NAME magic_enum
    GITHUB_REPOSITORY Neargye/magic_enum
    GIT_TAG v0.8.1 # Where `x.y.z` is the release version you want to use.
)

set(MAGIC_ENUM_OPT_INSTALL ON CACHE BOOL "Enable option to install Magic Enum" FORCE)
set(EXTERNAL "${PROJECT_SOURCE_DIR}/external")

find_package(ecat_master REQUIRED)

if(${COMPILE_EXAMPLE})
	# compile example
	add_subdirectory(src/ec_client_main)
endif()


if(${COMPILE_GUI})
	  # compile ec gui
	  add_subdirectory(src/ec_gui)
endif()

if(${COMPILE_GRAVITY_COMPENSATION})
	  # compile ec gui
	  add_subdirectory(src/ec_gravity_compensation)
endif()
