project(ec_gui)

# set qt to automatically run MOC and RCC pre-compilers
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(TIFF REQUIRED)

# find qt
find_package(Qt6UiTools 6.0 REQUIRED)
find_package(Qt6Widgets 6.0 REQUIRED)
find_package(Qt6Charts  6.0 REQUIRED)
find_package(Qt6PrintSupport  6.0 REQUIRED)
find_package(Qt6Network  6.0 REQUIRED)

OPTION(TEST_GUI "Testing GUI concepts" OFF)

IF(TEST_GUI)
    ADD_DEFINITIONS(-DTEST_GUI)
ENDIF(TEST_GUI)

include_directories(src/utils/
                    src/command/
                    src/pdo/
                    src/sdo/
                    src/wrapper/
                    src/network/
                    src/component/slider
                    src/component/plot
                    src/component/terminal)

# compile widget
add_library(ec_gui_lib STATIC
    src/ec_gui_start.cpp
    src/ec_gui_start_resources.qrc
    src/component/slider/ec_gui_slider.cpp
    src/component/slider/slider_window.cpp
    src/component/slider/slider_widget.cpp
    src/component/slider/wave_widget.cpp
    src/component/plot/qcustomplot.cpp
    src/component/terminal/ec_gui_terminal.cpp
    src/command/ec_gui_cmd.cpp
    src/pdo/ec_gui_pdo.cpp
    src/sdo/ec_gui_sdo.cpp
    src/wrapper/ec_gui_wrapper.cpp
    src/network/ec_gui_net.cpp
    )

target_link_libraries(ec_gui_lib
    PUBLIC
    Qt6::Widgets
    Qt6::UiTools
    Qt6::PrintSupport 
    Qt6::Network
    -lcurl
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${FREEIMAGE_LIBRARIES}
    ${TIFF_LIBRARIES}
    pthread
    ec_library
    uuid
    )

# compile main
add_executable(ec_gui src/ec_gui_main.cpp)

target_link_libraries(ec_gui ec_gui_lib)

install(TARGETS ec_gui
    DESTINATION bin
    COMPONENT applications)
