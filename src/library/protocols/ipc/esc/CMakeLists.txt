project(esc_pipes_library)
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
add_library(esc_pipes_library STATIC
            esc_iface.cpp
            motor/motor_iface.cpp
            imu/imu_iface.cpp
            ft/ft_iface.cpp
            pow/pow_iface.cpp)
        
target_include_directories(esc_pipes_library
    PUBLIC
     ${PROJECT_SOURCE_DIR}/include/
    motor/
    imu/
    ft/
    pow/
    ${ZeroMQ_INCLUDE_DIR} 
)

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

target_link_libraries(esc_pipes_library
    PUBLIC
    ecat_master::ecat_core${LIB_TYPE}
    ecat_master::ecat_master_pb
    ${ZeroMQ_LIBRARY}
    )


install(TARGETS esc_pipes_library
    DESTINATION lib)
