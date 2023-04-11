project(ec_simple_trajectory)
cmake_minimum_required(VERSION 3.0)

# std c++14 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# avoid installing catkin-related files
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)

OPTION(TEST_MAIN "Testing Main concepts" OFF)

# set includes
include_directories(
    ${PROJECT_SOURCE_DIR}/../../protocols/udp
    ${PROJECT_SOURCE_DIR}/../../utils
    )

find_package(Boost REQUIRED system)

find_package(yaml-cpp REQUIRED)

IF(TEST_MAIN)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_MAIN)

# compile main
add_executable(ec_simple_trajectory
               src/ec_simple_trajectory.cpp
               ${PROJECT_SOURCE_DIR}/../../protocols/udp/ec_udp_comm.cpp
               ${PROJECT_SOURCE_DIR}/../../utils/ec_utils.cpp)
               
target_include_directories(ec_simple_trajectory
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_simple_trajectory
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    )

install(TARGETS ec_simple_trajectory
    DESTINATION bin)
