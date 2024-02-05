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

# compile ecat common library
add_library(ec_common_library STATIC
            common/zmq/ec_zmq_cmd.cpp
            common/zmq/ec_zmq_pdo.cpp
            common/pipe/ec_pipe_pdo.cpp
            common/ec_cmd.cpp
            common/ec_pdo.cpp
            logger/ec_logger.cpp
            ec_iface.cpp)
 
target_include_directories(ec_common_library
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EXTERNAL}/spdlog/include>
    ${ZeroMQ_INCLUDE_DIR}
)

target_link_libraries(ec_common_library
    PUBLIC
    ecat_master::ecat_core${LIB_TYPE}
    ecat_master::ecat_master_pb
    ecat_master::msgpck_proto
    ${ZeroMQ_LIBRARY}
    matlogger2::matlogger2
    magic_enum::magic_enum
    )


# compile ecat library test
add_library(${PROJECT_NAME} STATIC
            protocols/udp/ec_udp.cpp
            protocols/tcp/ec_tcp.cpp
            protocols/ipc/zipc/ec_zipc.cpp
            protocols/ipc/iddp/ec_iddp.cpp
            utils/ec_utils.cpp
            utils/ec_common_step.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include/ecat-client-advr/${PROJECT_NAME}>")            
            

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_common_library
    )
    

install(DIRECTORY include/
    DESTINATION include/ecat-client-advr/${PROJECT_NAME}
    COMPONENT headers
    FILES_MATCHING PATTERN "*.h*"
    PATTERN "cmake" EXCLUDE
    )
    
install(TARGETS ec_common_library
    EXPORT ecat_client_advrTargets
    DESTINATION lib
    COMPONENT libraries)

install(TARGETS ${PROJECT_NAME}
    EXPORT ecat_client_advrTargets
    DESTINATION lib
    COMPONENT libraries)
