project(ec_common_library)
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
add_library(${PROJECT_NAME} STATIC
            zmq/ec_zmq_cmd.cpp
            zmq/ec_zmq_pdo.cpp
            pipe/ec_pipe_pdo.cpp
            ec_cmd.cpp
            ec_pdo.cpp)
            

target_include_directories(${PROJECT_NAME}
    PUBLIC
     ${CMAKE_SOURCE_DIR}/src/library/include/
    ${ZeroMQ_INCLUDE_DIR} 
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ecat_master::ecat_core${LIB_TYPE}
    ecat_master::ecat_master_pb
    ecat_master::msgpck_proto
    ${ZeroMQ_LIBRARY}
    )


install(TARGETS ${PROJECT_NAME}
    DESTINATION lib)
