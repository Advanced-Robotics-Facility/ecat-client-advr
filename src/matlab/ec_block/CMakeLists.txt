# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(ec_block LANGUAGES CXX VERSION 0.1)

# Utility for defining install folders
include(GNUInstallDirs)

# Tweak linker flags in Linux.
# Matlab is very strict on missing symbols and by default ld do not warn if
# something is missing.
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
    endif()
endif()

# Find the needed BlockFactory components:
# - "Core" contains the core classes such as Block and Signal
# - "Simulink" is required at runtime for loading the library from Simulink
find_package(BlockFactory
    REQUIRED COMPONENTS Core
    OPTIONAL_COMPONENTS Simulink)
    
find_package(XBotInterface REQUIRED)

include_directories(include/common/)

add_library(ec_block_utils SHARED
            src/common/ec_block_utils.cpp)

# Link the library with the Core component containing the core classes
target_link_libraries(ec_block_utils PUBLIC ec_library PRIVATE ${XBotInterface_LIBRARIES})
            
# Setup the include directories
target_include_directories(ec_block_utils 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/common>)

include(cmake/manager.cmake)
#include(Utilities)
#include(Sensors)
