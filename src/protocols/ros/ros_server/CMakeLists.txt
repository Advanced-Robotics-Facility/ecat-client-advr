find_package(ecat_master REQUIRED)
find_package(yaml-cpp QUIET)
find_package(catkin REQUIRED COMPONENTS roscpp ec_msgs ec_srvs kdl_parser visualization_msgs)

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
        
find_package(XBotInterface REQUIRED)

        
set(COMMON_LIBS
    yaml-cpp
)


get_target_property(soem_INCLUDE_DIRS soem${ECAT_LIB_TYPE}
  INTERFACE_INCLUDE_DIRECTORIES)
  
get_target_property(ecat_INCLUDE_DIRS ecat_master::ecat_core${ECAT_LIB_TYPE}
  INTERFACE_INCLUDE_DIRECTORIES)

include_directories(ros 
                    slave_setup 
                    command
                    ${ZeroMQ_INCLUDE_DIR} 
                    ${catkin_INCLUDE_DIRS}
                    ${soem_INCLUDE_DIRS}
                    ${ecat_INCLUDE_DIRS})

#ec_client_main
add_executable(ec_client_main
    executor/ec_client_main.cpp
    ros/ec_client_ros.cpp
    ros/slave_ros.cpp
    ros/motor_ros.cpp
    ros/pow_ros.cpp
    ros/imu_ros.cpp
    ros/ft_ros.cpp
    ros/rviz/robot_viz.cpp
    slave_setup/motor_setup.cpp
    command/ec_client_cmd.cpp
    command/ec_client_pdo.cpp) 
    
add_dependencies(ec_client_main ec_msgs_generate_messages ec_srvs_generate_messages)
target_link_libraries(ec_client_main PUBLIC ${ZeroMQ_LIBRARY} ecat_master::ecat_master_pb ${catkin_LIBRARIES} -lyaml-cpp ${XBotInterface_LIBRARIES})


#ec_simulator

add_executable(ec_sim
    ec_simulator/ec_simulator.cpp
    ec_simulator/rt_thread.cpp
    ec_simulator/nrt_thread.cpp
    ec_simulator/data_cb_handler.cpp
    command/ec_client_cmd.cpp
)

if(Xenomai_FOUND)
    xenomai_target_bootstrap(ec_sim SKINS posix )
endif()

target_link_libraries(ec_sim
                      PUBLIC 
                      ${ZeroMQ_LIBRARY} 
                      ecat_master::ecat_master_pb
                      ecat_master::ecat_core${ECAT_LIB_TYPE}
                      ecat_master::tools_utils
                      -lyaml-cpp
                      ${catkin_LIBRARIES}
                      zmq)

#ec_client_library

add_library(ec_client_library SHARED
            command/ec_client_cmd.cpp
            command/ec_client_pdo.cpp) 
    
target_link_libraries(ec_client_library PUBLIC zmq  PRIVATE ecat_master::ecat_master_pb)

set(EC_CLIENT_LIB_VERSION_MAJOR 0)
set(EC_CLIENT_LIB_VERSION_MINOR 1)
set(EC_CLIENT_LIB_VERSION_PATCH 1)
set(EC_CLIENT_LIB_VERSION_STRING ${EC_CLIENT_LIB_VERSION_MAJOR}.${EC_CLIENT_LIB_VERSION_MINOR}.${EC_CLIENT_LIB_VERSION_PATCH})

#set_target_properties(ec_client_library PROPERTIES VERSION ${EC_CLIENT_LIB_VERSION_STRING}
#                                                   SOVERSION ${EC_CLIENT_LIB_VERSION_MAJOR})

install(DIRECTORY command/
        DESTINATION include/ec_xbot2_client/ec_client_library
        COMPONENT headers
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "cmake" EXCLUDE
        )

install(
    TARGETS ec_client_main
    DESTINATION bin
    COMPONENT applications
    )
    
install(
    TARGETS ec_sim
    DESTINATION bin
    COMPONENT applications
    )
    
install(
    TARGETS ec_client_library
    EXPORT ec_xbot2_clientTargets
    DESTINATION lib
    COMPONENT libraries
    )

