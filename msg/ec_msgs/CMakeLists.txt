cmake_minimum_required(VERSION 3.0)
project (ec_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
    FILES
    CmdStatus.msg
    SlaveInfo.msg
    SlaveCmdInfo.msg
    SlaveDescription.msg
    MotorPDO.msg
    PowPDO.msg
    ImuPDO.msg
    FtPDO.msg
    HandStatus.msg
    HandCmd.msg
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package()
