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
            ../../ec_iface.cpp
            zmq/ec_zmq_cmd.cpp
            zmq/ec_zmq_pdo.cpp
            pipe/ec_pipe_pdo.cpp
            ec_cmd.cpp
            ec_pdo.cpp)
            



