# ===========
# C++ LIBRARY
# ===========

######## RobotManager Library ################################
include_directories(../include/manager/)
# Create the plugin library. This must be a SHARED library.
add_library(RobotManager SHARED
    src/manager/robot_manager.cpp
    src/manager/reading.cpp
    src/manager/reference.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(RobotManager PROPERTIES
    OUTPUT_NAME "RobotManager")

# Link the library with the Core component containing the core classes
target_link_libraries(RobotManager PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ec_block_utils)

# Setup the include directories
target_include_directories(RobotManager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

######## RobotManager Library ################################


######## ModelManager Library ################################
   
# Create the plugin library. This must be a SHARED library.

# add_library(ModelManager SHARED
#     include/XBotBlock/Managers/ModelManager.h
#     include/XBotBlock/Managers/MechanicalLimits.h
#     include/XBotBlock/Managers/Readings.h
#     include/XBotBlock/Managers/References.h
#     include/XBotBlock/Managers/FloatingBase.h
#     include/XBotBlock/Managers/SensorManager.h
#     include/XBotBlock/Managers/Gravity.h
#     src/XBotBlock/Managers/ModelManager.cpp
#     src/XBotBlock/Managers/MechanicalLimits.cpp
#     src/XBotBlock/Managers/Readings.cpp
#     src/XBotBlock/Managers/References.cpp
#     src/XBotBlock/Managers/FloatingBase.cpp
#     src/XBotBlock/Managers/Gravity.cpp
#     src/XBotBlock/Managers/SensorManager.cpp)
# 
# # Manually set the name of the output library. This is not required and it
# # is done only for sake of clarity.
# set_target_properties(ModelManager PROPERTIES
#     OUTPUT_NAME "ModelManager")
# 
# # Link the library with the Core component containing the core classes
# target_link_libraries(ModelManager PRIVATE
#     BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)
# 
# # Setup the include directories
# target_include_directories(ModelManager PRIVATE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# 
#  set_target_properties(ModelManager PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
#  install(TARGETS ModelManager RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## ModelManager Library ################################


