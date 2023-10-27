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

# compile ecat library test
add_library(ec_library STATIC
            ${PROJECT_SOURCE_DIR}/protocols/common/zmq/ec_zmq_cmd.cpp
            ${PROJECT_SOURCE_DIR}/protocols/common/ec_cmd.cpp
            ${PROJECT_SOURCE_DIR}/protocols/tcp/ec_tcp.cpp
            ${PROJECT_SOURCE_DIR}/protocols/udp/ec_udp.cpp
            ${PROJECT_SOURCE_DIR}/protocols/ipc/iddp/ec_iddp.cpp
            ${PROJECT_SOURCE_DIR}/utils/ec_utils.cpp
            ${PROJECT_SOURCE_DIR}/logger/ec_logger.cpp)

target_include_directories(ec_library
    PUBLIC
    include/
    utils/
    logger/
    protocols/common/
    protocols/common/zmq/
    PRIVATE
    ${EXTERNAL}/spdlog/include
    ${ZeroMQ_INCLUDE_DIR} 
)

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

target_link_libraries(ec_library
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::ecat_core${LIB_TYPE}
    ecat_master::ecat_master_pb
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    matlogger2::matlogger2
    ${ZeroMQ_LIBRARY}
    )


install(TARGETS ec_library
    DESTINATION lib)
