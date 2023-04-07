# std c++14 flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

# fmt header only mode to avoid linking
find_package(fmt REQUIRED)

find_package(yaml-cpp QUIET)


# find required packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ec_msgs
    ec_srvs
    )


find_package(XBotInterface REQUIRED)

# set includes
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_INSTALL_PREFIX}/include
    )

# set qt to automatically run MOC and RCC pre-compilers
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# find qt
find_package(Qt5UiTools 5.9 REQUIRED)
find_package(Qt5Widgets 5.9 REQUIRED)
find_package(Qt5Charts  5.9 REQUIRED)
find_package(Qt5PrintSupport  5.9 REQUIRED)
find_package(robot_monitoring REQUIRED)

# compile widget
add_library(xbot_rob_mon_plugin_ecat SHARED
    slave_resources.qrc
    slave_cmd_widget.cpp
    joint_widget.cpp
    slider_widget.cpp
    sliders_widget.cpp
    slave_widget.cpp
    )
#ADD_LIBRARY(robot_monitoring STATIC IMPORTED)
#SET_TARGET_PROPERTIES(robot_monitoring PROPERTIES IMPORTED_LOCATION ${CMAKE_INSTALL_PREFIX}/lib/librobot_monitoring.so)

target_link_libraries(xbot_rob_mon_plugin_ecat
    PUBLIC
    Qt5::Widgets
    Qt5::UiTools
    fmt::fmt
    -lyaml-cpp
    -lcurl
    ${catkin_LIBRARIES}
    uuid
    ${XBotInterface_LIBRARIES}
    ${robot_monitoring_LIBRARIES}
    )
    
add_dependencies(xbot_rob_mon_plugin_ecat ec_msgs_generate_messages ec_srvs_generate_messages)

target_include_directories(xbot_rob_mon_plugin_ecat
    PUBLIC
    ${robot_monitoring_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

install(TARGETS xbot_rob_mon_plugin_ecat
    DESTINATION lib)
