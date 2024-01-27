project(ec_library)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#compile ec_library
add_subdirectory(protocols/common)

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
add_library(${PROJECT_NAME} STATIC
            ${PROJECT_SOURCE_DIR}/protocols/udp/ec_udp.cpp
            ${PROJECT_SOURCE_DIR}/protocols/tcp/ec_tcp.cpp
            ${PROJECT_SOURCE_DIR}/protocols/ipc/zipc/ec_zipc.cpp
            ${PROJECT_SOURCE_DIR}/protocols/ipc/iddp/ec_iddp.cpp
            ${PROJECT_SOURCE_DIR}/utils/ec_utils.cpp
            ${PROJECT_SOURCE_DIR}/logger/ec_logger.cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    include/
    PRIVATE
    ${EXTERNAL}/spdlog/include
    ${ZeroMQ_INCLUDE_DIR} 
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ec_common_library
    magic_enum::magic_enum
    matlogger2::matlogger2
    )


install(TARGETS ${PROJECT_NAME}
    DESTINATION lib)
