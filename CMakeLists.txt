cmake_minimum_required(VERSION 3.0)


project(ecat-client-advr)

# c++17 support
set(CMAKE_CXX_STANDARD 17)

option(EC_CLIENT_ENABLE "Enable to compile the ROS etherCAT client" ON)
option(EC_CLIENT_ENABLE_GUI "Enable ROS etherCAT client GUI" OFF)
option(EC_CLIENT_UDP "Enable to compile the UDP etherCAT client and GUI" ON)

set(USE_ECAT_CORE ON CACHE BOOL "Use ECAT CORE LIBRARY" FORCE)
set(ECAT_LIB_TYPE _nrt)

#compile messages
add_subdirectory(msg/ec_msgs)

#compile services
add_subdirectory(srv/ec_srvs)

if(${EC_CLIENT_ENABLE})
	# compile ec client
	add_subdirectory(src/ec_client_ros/ec_client)
endif()


if(${EC_CLIENT_ENABLE_GUI})
	add_subdirectory(src/ec_client_ros/ec_gui)
endif()

if(${EC_CLIENT_UDP})
	add_subdirectory(src/ec_client_udp)
endif()
