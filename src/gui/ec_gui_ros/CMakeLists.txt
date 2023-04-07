project(ec_gui)
cmake_minimum_required(VERSION 3.0)

# std c++14 flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

# fmt header only mode to avoid linking
find_package(fmt REQUIRED)

# find rviz
find_package(rviz REQUIRED)

find_package(yaml-cpp QUIET)

find_package(TIFF REQUIRED)

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

add_subdirectory(src/slave/)

# compile widget
add_library(ec_gui STATIC
    src/ec_gui.cpp
    src/master/master_resources.qrc
    src/master/master_cmd_widget.cpp
    src/rviz/rviz_widget.cpp
    src/ec_gui_start.cpp
    src/ec_gui_start_resources.qrc
    src/pdo/pdo_resources.qrc
    src/pdo/pdo_widget.cpp
    )

target_link_libraries(ec_gui
    PUBLIC
    Qt5::Widgets
    Qt5::UiTools
    fmt::fmt
    -lyaml-cpp
    -lcurl
    ${catkin_LIBRARIES}
    ${rviz_LIBRARIES}
    ${FREEIMAGE_LIBRARIES}
    ${TIFF_LIBRARIES}
    uuid
    ${XBotInterface_LIBRARIES}
    xbot_rob_mon_plugin_ecat
    )

target_include_directories(ec_gui
    PUBLIC
    ${rviz_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

# compile main
add_executable(ec_gui_main src/ec_gui_main.cpp)
target_link_libraries(ec_gui_main ec_gui)
install(TARGETS ec_gui_main
    DESTINATION bin
    COMPONENT applications)
