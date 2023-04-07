project(ec_gravity_compensation)
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
    ${PROJECT_SOURCE_DIR}/../UDP_Client
    ${PROJECT_SOURCE_DIR}/../Utils
    )

find_package(Boost REQUIRED system)

find_package(yaml-cpp REQUIRED)

find_package(XBotInterface REQUIRED)

IF(TEST_MAIN)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_MAIN)

# compile gravity_compensation
add_executable(ec_gravity_compensation
               src/ec_gravity_compensation.cpp
               ${PROJECT_SOURCE_DIR}/../UDP_Client/client.cpp
               ${PROJECT_SOURCE_DIR}/../Utils/ec_client_utils.cpp)
               
target_include_directories(ec_gravity_compensation
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_gravity_compensation
    PRIVATE 
    ${XBotInterface_LIBRARIES}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    )
    

install(TARGETS ec_gravity_compensation
    DESTINATION bin)

