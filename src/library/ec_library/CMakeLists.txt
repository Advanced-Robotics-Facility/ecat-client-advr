project(ec_library)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## load in pkg-config support
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )
        
OPTION(TEST_LIBRARY "Testing Library" OFF)
IF(TEST_LIBRARY)
    ADD_DEFINITIONS(-DTEST_LIBRARY)
ENDIF(TEST_LIBRARY)

# compile ecat mechanism library
add_library(ec_api_library STATIC
            src/mechanism/boost/ec_boost_cmd.cpp
            src/mechanism/boost/ec_boost_pdo.cpp
            src/mechanism/zmq/ec_repl_cmd.cpp
            src/mechanism/zmq/ec_zmq_cmd.cpp
            src/mechanism/zmq/ec_zmq_pdo.cpp
            src/mechanism/pipe/ec_pipe_pdo.cpp
            src/mechanism/common/ec_pdo.cpp
            src/logger/ec_logger.cpp
            src/api/ec_iface.cpp)
 
target_include_directories(ec_api_library
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EXTERNAL}/spdlog/include>
    ${ZeroMQ_INCLUDE_DIR}
)

target_link_libraries(ec_api_library
    PUBLIC
    ecat_master::ecat_core${LIB_TYPE}
    ecat_master::ecat_master_pb
    ecat_master::msgpck_proto
    ${ZeroMQ_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    matlogger2::matlogger2
    magic_enum::magic_enum
    )


# compile ecat library test
add_library(${PROJECT_NAME} STATIC
            src/protocol/udp/ec_udp.cpp
            src/protocol/tcp/ec_tcp.cpp
            src/protocol/ipc/zipc/ec_zipc.cpp
            src/protocol/ipc/iddp/ec_iddp.cpp
            src/utils/ec_utils.cpp
            src/utils/ec_wrapper.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include/ecat-client-advr/${PROJECT_NAME}>")            
            

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    pthread
    ec_api_library
    )
    

install(DIRECTORY include/
    DESTINATION include/ecat-client-advr/${PROJECT_NAME}
    COMPONENT headers
    FILES_MATCHING PATTERN "*.h*"
    PATTERN "cmake" EXCLUDE
    )
    
install(TARGETS ec_api_library
    EXPORT ecat_client_advrTargets
    DESTINATION lib
    COMPONENT libraries)

install(TARGETS ${PROJECT_NAME}
    EXPORT ecat_client_advrTargets
    DESTINATION lib
    COMPONENT libraries)
