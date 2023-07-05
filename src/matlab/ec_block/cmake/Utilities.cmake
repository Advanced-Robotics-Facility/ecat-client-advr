# ===========
# C++ LIBRARY
# ===========

######## Kinematics Library ################################
 
# Create the plugin library. This must be a SHARED library.
add_library(Kinematics SHARED
    include/XBotBlock/Utilities/Kinematics.h
    include/XBotBlock/Utilities/Pose.h
    include/XBotBlock/Utilities/Twist.h
    include/XBotBlock/Utilities/COM.h
    src/XBotBlock/Utilities/Kinematics.cpp
    src/XBotBlock/Utilities/Pose.cpp
    src/XBotBlock/Utilities/Twist.cpp
    src/XBotBlock/Utilities/COM.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(Kinematics PROPERTIES
    OUTPUT_NAME "Kinematics")

# Link the library with the Core component containing the core classes
target_link_libraries(Kinematics PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)

# Setup the include directories
target_include_directories(Kinematics PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set_target_properties(Kinematics PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Kinematics RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## Kinematics Library ################################

######## Jacobian Library ################################

# Create the plugin library. This must be a SHARED library.
add_library(Jacobian SHARED
    include/XBotBlock/Utilities/Jacobian.h
    src/XBotBlock/Utilities/Jacobian.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(Jacobian PROPERTIES
    OUTPUT_NAME "Jacobian")

# Link the library with the Core component containing the core classes
target_link_libraries(Jacobian PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)

# Setup the include directories
target_include_directories(Jacobian PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set_target_properties(Jacobian PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Jacobian RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## Jacobian Library ################################
  
######## Dynamics Library ################################

# Create the plugin library. This must be a SHARED library.
add_library(Dynamics SHARED
    include/XBotBlock/Utilities/Dynamics.h
    src/XBotBlock/Utilities/Dynamics.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(Dynamics PROPERTIES
    OUTPUT_NAME "Dynamics")

# Link the library with the Core component containing the core classes
target_link_libraries(Dynamics PRIVATE
    BlockFactory::Core ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} XBotBlock_Utils)

# Setup the include directories
target_include_directories(Dynamics PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
   
set_target_properties(Dynamics PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Dynamics RUNTIME LIBRARY DESTINATION lib COMPONENT libraries)
    
######## Dynamics Library ################################
    
