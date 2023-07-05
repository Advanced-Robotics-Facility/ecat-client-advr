# ===========
# C++ LIBRARY
# ===========

######## IMU Library ################################
 
# Create the plugin library. This must be a SHARED library.
add_library(IMU SHARED
    include/XBotBlock/Sensors/IMU.h
    src/XBotBlock/Sensors/IMU.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(IMU PROPERTIES
    OUTPUT_NAME "IMU")

# Link the library with the Core component containing the core classes
target_link_libraries(IMU PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)

# Setup the include directories
target_include_directories(IMU PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set_target_properties(IMU PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS IMU RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## IMU Library ################################

######## ForceTorque Library ################################
 
# Create the plugin library. This must be a SHARED library.
add_library(ForceTorque SHARED
    include/XBotBlock/Sensors/ForceTorque.h
    src/XBotBlock/Sensors/ForceTorque.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(ForceTorque PROPERTIES
    OUTPUT_NAME "ForceTorque")

# Link the library with the Core component containing the core classes
target_link_libraries(ForceTorque PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)

# Setup the include directories
target_include_directories(ForceTorque PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set_target_properties(ForceTorque PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ForceTorque RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## ForceTorque Library ################################
