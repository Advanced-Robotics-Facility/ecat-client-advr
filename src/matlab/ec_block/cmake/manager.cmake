# ===========
# C++ LIBRARY
# ===========

######## EcManager Library ################################
include_directories(../include/manager/)
include_directories(../include/sensor/)
include_directories(../include/power/)
# Create the plugin library. This must be a SHARED library.
add_library(EcManager SHARED
    src/manager/ec_manager.cpp
    src/manager/ec_reading.cpp
    src/manager/ec_reference.cpp
    src/sensor/ec_imu.cpp
    src/sensor/ec_ft.cpp
    src/power/ec_pow.cpp
    )

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


