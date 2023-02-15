project(ec_gui)
cmake_minimum_required(VERSION 3.0)

# std c++14 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

find_package(yaml-cpp QUIET)

find_package(TIFF REQUIRED)


OPTION(TEST_GUI "Testing GUI concepts" OFF)
    
# set includes
include_directories(
    ../ec_client_library/command
    ${PROJECT_SOURCE_DIR}/../UDP_Client
    ${PROJECT_SOURCE_DIR}/../Utils
    )

# set qt to automatically run MOC and RCC pre-compilers
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# find qt
find_package(Qt5UiTools 5.9 REQUIRED)
find_package(Qt5Widgets 5.9 REQUIRED)
find_package(Qt5Charts  5.9 REQUIRED)
find_package(Qt5PrintSupport  5.9 REQUIRED)

find_package(Boost REQUIRED system)

set(COMMON_LIBS
    yaml-cpp
)

# compile widget
add_library(ec_gui_lib STATIC
    src/ec_gui_start.cpp
    src/ec_gui_start_resources.qrc
    src/slider_widget.cpp
    src/slider_widget_calib.cpp
    ${PROJECT_SOURCE_DIR}/../UDP_Client/client.cpp
    ${PROJECT_SOURCE_DIR}/../Utils/ec_client_utils.cpp
    )

target_include_directories(ec_gui_lib
    PRIVATE
    ${EXTERNAL}/spdlog/include
)
    
target_link_libraries(ec_gui_lib
    PUBLIC
    Qt5::Widgets
    Qt5::UiTools
    -lcurl
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${FREEIMAGE_LIBRARIES}
    ${TIFF_LIBRARIES}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    uuid
    )

IF(TEST_GUI)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_GUI)


# compile main
add_executable(ec_gui_mirror src/ec_gui_main.cpp)

target_include_directories(ec_gui_mirror
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_gui_mirror ec_gui_lib)
install(TARGETS ec_gui_mirror
    DESTINATION bin)


install(TARGETS ec_gui_mirror
    DESTINATION bin)
