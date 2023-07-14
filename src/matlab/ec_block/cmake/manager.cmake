# ===========
# C++ LIBRARY
# ===========

######## EcManager Library ################################
include_directories(../include/manager/)
# Create the plugin library. This must be a SHARED library.
add_library(EcManager SHARED
    src/manager/ec_manager.cpp
    src/manager/reading.cpp
    src/manager/reference.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(EcManager PROPERTIES
    OUTPUT_NAME "EcManager")

# Link the library with the Core component containing the core classes
target_link_libraries(EcManager PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ec_block_utils)

# Setup the include directories
target_include_directories(EcManager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    
install(TARGETS EcManager
    DESTINATION lib)    

######## EcManager Library ################################


