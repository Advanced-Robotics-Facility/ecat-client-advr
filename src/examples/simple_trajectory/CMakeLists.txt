project(ec_client_main)
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

IF(TEST_MAIN)
    ADD_DEFINITIONS(-DTEST)
ENDIF(TEST_MAIN)

# compile main
add_executable(ec_client_mirror
               src/ec_client_main.cpp
               ${PROJECT_SOURCE_DIR}/../UDP_Client/client.cpp
               ${PROJECT_SOURCE_DIR}/../Utils/ec_client_utils.cpp)
               
target_include_directories(ec_client_mirror
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_client_mirror
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    )
    
# compile ping pong test
add_executable(ping_pong_test
               src/ping_pong_test.cpp
               ${PROJECT_SOURCE_DIR}/../UDP_Client/client.cpp
               ${PROJECT_SOURCE_DIR}/../Utils/ec_client_utils.cpp)
 
 
target_include_directories(ping_pong_test
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ping_pong_test
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    )

# compile frequency test
add_executable(ec_client_freq_test
               src/ec_client_freq_test.cpp
               ${PROJECT_SOURCE_DIR}/../UDP_Client/client.cpp
               ${PROJECT_SOURCE_DIR}/../Utils/ec_client_utils.cpp)
 
 
target_include_directories(ec_client_freq_test
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_client_freq_test
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    )


install(TARGETS ec_client_mirror
    DESTINATION bin)
    
install(TARGETS ping_pong_test
    DESTINATION bin)

install(TARGETS ec_client_freq_test
    DESTINATION bin)
