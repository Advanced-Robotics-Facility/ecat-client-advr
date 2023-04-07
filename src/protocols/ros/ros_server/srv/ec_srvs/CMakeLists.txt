cmake_minimum_required(VERSION 3.0)
project (ec_srvs)
#include("${CMAKE_INSTALL_PREFIX}/share/zmq_client_msg/cmake/zmq_client_msg-msg-paths.cmake")
find_package(catkin REQUIRED COMPONENTS ec_msgs std_msgs message_generation)
add_service_files(
    FILES
    GetGainsCurrentInfo.srv
    GetSlaveInfo.srv
    GetSlaveInfoStartMaster.srv
    GetSlaveSdo.srv
    PrintMotorInfo.srv
    SelectSlave.srv
    SetMotorImpGains.srv
    SetMotorPosVelGains.srv
    SetMotorLimits.srv
    SetSlaveSdo.srv
    SetSlaveCurrent.srv
    SetSlavesFirmware.srv
    SetMotorConfigFile.srv
    SetMotorCtrlValue.srv
    SetMotorHomingTrj.srv
    SetMotorPeriodTrj.srv
    SetMotorSmoothTrj.srv
)

generate_messages(DEPENDENCIES ec_msgs std_msgs)
catkin_package()
