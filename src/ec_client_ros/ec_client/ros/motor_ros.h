#ifndef __MOTOR_ROS__
#define __MOTOR_ROS__

#include "motor_setup.h"
#include "ec_client_cmd.h"
#include "ec_client_pdo.h"

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "ec_msgs/SlaveCmdInfo.h"
#include "ec_msgs/CmdStatus.h"
#include "ec_msgs/MotorPDO.h"

#include "ec_srvs/SetSlaveCurrent.h"
#include "ec_srvs/SelectSlave.h"
#include "ec_srvs/SetMotorImpGains.h"
#include "ec_srvs/SetMotorPosVelGains.h"
#include "ec_srvs/SetMotorConfigFile.h"
#include "ec_srvs/GetGainsCurrentInfo.h"
#include "ec_srvs/GetSlaveInfo.h"
#include "ec_srvs/GetSlaveInfoStartMaster.h"
#include "ec_srvs/PrintMotorInfo.h"
#include "ec_srvs/SetMotorLimits.h"
#include "ec_srvs/SetSlavesFirmware.h"
#include "ec_srvs/SetMotorCtrlValue.h"
#include "ec_srvs/SetMotorHomingTrj.h"
#include "ec_srvs/SetMotorPeriodTrj.h"
#include "ec_srvs/SetMotorSmoothTrj.h"
#include "rviz/robot_viz.h"
#include <std_srvs/Empty.h>


class Motor_ROS
{
public:
    
    typedef std::shared_ptr<Motor_ROS> Ptr;
 
    /**
    * @brief Constructor of Motor ROS Class
    * It needs of shared pointer to the ROS Node Handle for the topics/services.
    * 
    * @param node p_node: ROS Node Handle.
    */
    Motor_ROS(std::shared_ptr<ros::NodeHandle> node);
    

    /**
    * @brief Destructor of Motor ROS Class
    * 
    */
    ~Motor_ROS(){
    };
    
    
    /**
    * @brief Method to set the EC_Client CMD shared pointer responsible to send the ZMQ command and receive a reply.
    * 
    * @param ec_client_cmd p_ec_client_cmd: Smart pointer to EC Client CMD class.
    */
    void set_ec_client_cmd(EC_Client_CMD::Ptr ec_client_cmd);
    
    /**
    * @brief Method to set the status of the callback externally.
    * 
    * @param status_callback p_status_callback: boolean value of callback status.
    */
    void set_status_callback(bool status_callback);
    
    /**
    * @brief Method that return the status of the callbacks from the slaves.
    * 
    * @return bool
    */
    bool get_status_callback();
    
    /**
    * @brief This method allows to set the XbotCore Presence to forbid the services.
    * 
    * @param xbotcorepresence p_xbotcorepresence: set boolean value for the XBotCore presence.
    */
    void set_xbotcorepresence(bool xbotcorepresence);
    
    /**
    * @brief Return motor map of the motors configured with the low level configuration file.
    * 
    * @return std::map< std::__cxx11::string, Motor_Setup::Ptr >
    */
    std::map<std::string, Motor_Setup::Ptr> get_motor_map_setup();
    
    /**
    * @brief Set motor map of the motors configured with the low level configuration file...
    * 
    * @param motor_map_setup p_motor_map_setup:...
    */
    void set_motor_map_setup(std::map<std::string, Motor_Setup::Ptr> motor_map_setup);
    
    /**
    * @brief Return the map of motors with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @return std::map< std::__cxx11::string, EC_Client_PDO::Ptr >
    */
    std::map<std::string,EC_Client_PDO::Ptr> get_ec_client_pdo_map();
    
    /**
    * @brief Set the map of motors with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @param ec_client_pdo_map p_ec_client_pdo_map: Map of slave name and smart pointer of EC Client PDO class.
    */
    void set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr> ec_client_pdo_map);
    
    /**
    * @brief Set the model interface of the robot.
    * 
    * @param model p_model: Model of the robot.
    */
    void set_model_interface(XBot::ModelInterface::Ptr model);
    
    /**
    * @brief Set the smart pointer of RobotViz class.
    * 
    * @param robotviz p_robotviz: RobotViz class for publishing the robot pose.
    */
    void set_robotviz(std::shared_ptr<RobotViz> robotviz);
    
    /**
    * @brief Clear PDO motors message.
    * 
    */
    void clear_motors_pdo_message();
    
    /**
    * @brief Clear Joint Map
    * 
    */
    void clear_joint_map();
    
    /**
    * @brief Publish robot pose.
    * 
    */
    void pub_robot();
    
    /**
    * @brief Publish motors PDOs.
    * 
    * @return bool
    */
    bool pub_motors_pdo();
    
    /**
    * @brief Return the string of the Fault from a numeric value.
    * 
    * @param fault p_fault: Numeric value of the fault
    * @return std::__cxx11::string
    */
    std::string handle_fault(uint32_t fault);

    /**
    * @brief This method allows to prepare the message for printing out the motors configuration.
    * 
    * @param slave_name p_slave_name: Slave name for getting the motor configuration.
    * @param motor_setup p_motor_setup: Smart pointer of Motor setup class that has the information related on PIDs, Max Current and etc... 
    * @param slaves_descr p_slaves_descr: Message to send like reply of the service.
    */
    void prepare_motor_message(std::string slave_name,Motor_Setup::Ptr motor_setup,std::vector<ec_msgs::SlaveDescription> &slaves_descr);
    
    /**
    * @brief Service to print the motors configuration.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: Return the messages of Slaves Description.
    * @return bool
    */
    bool print_motors_info(ec_srvs::PrintMotorInfo::Request  &req,ec_srvs::PrintMotorInfo::Response &res);

    /**
    * @brief Service to stop the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool stop_motors(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to start the motors using the position control.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool start_motors_posmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to start the motors using the velocity control.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool start_motors_velmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to start the motors using the impedance control.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool start_motors_impmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to release brakes of the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool release_motors_brake(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to engage brakes of the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool engage_motors_brake(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to get the current control information (PID and Max Current) of the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool get_motors_ampere_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to get the mechanical limits of the motoros, minimum and maximum position, maximum velocity and torque.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool get_motors_limits_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to get the PID information of the motors already started.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool get_motors_gains_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to set the position control of the motors using the SDO or PDO.
    * 
    * @param req p_req: Slave Name, PID, SOD/PDO type.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_posgains(ec_srvs::SetMotorPosVelGains::Request  &req,ec_srvs::SetMotorPosVelGains::Response &res);
    
    /**
    * @brief Service to set the velocity control of the motors using the SDO or PDO.
    * 
    * @param req p_req: Slave Name, PID, SOD/PDO type.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_velgains(ec_srvs::SetMotorPosVelGains::Request  &req,ec_srvs::SetMotorPosVelGains::Response &res);
    
    /**
    * @brief Service to set the impedance control of the motors using the SDO or PDO.
    * 
    * @param req p_req: Slave Name, PID, SOD/PDO type.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_impgains(ec_srvs::SetMotorImpGains::Request  &req,ec_srvs::SetMotorImpGains::Response &res);
    
    /**
    * @brief Service to set the mechanical limits of the motors.
    * 
    * @param req p_req: Slave Name, minimum and maximum position, maximum velocity and maximum torque.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_limits(ec_srvs::SetMotorLimits::Request  &req,ec_srvs::SetMotorLimits::Response &res);
    
    /**
    * @brief Service to set the max current of saturation of the motors.
    * 
    * @param req p_req: Motor name and max current vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_max_ampere(ec_srvs::SetSlaveCurrent::Request  &req,ec_srvs::SetSlaveCurrent::Response &res);
    
    /**
    * @brief Service to set the position[rad] of the motors
    * 
    * @param req p_req: Motor name and position vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_position(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    
    /**
    * @brief Service to set the velocity[rad/s] of the motors
    * 
    * @param req p_req: Motor name and velocity vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_velocity(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    
    /**
    * @brief Service to set the torque[Nm] of the motors
    * 
    * @param req p_req: Motor name and torque vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_torque(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    

    /**
    * @brief Service to set the current[A] of the motors.
    * 
    * @param req p_req: Motor name and current vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_amperage(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    
    /**
    * @brief Service to set the homing position[rad] of the motors.
    * 
    * @param req p_req: Motor name and homing position vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_homing_position(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    
    /**
    * @brief Service to set ON/OFF the FAN of the motors.
    * 
    * @param req p_req: Motor name and fan(ON/OFF) vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_fan(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);
    
    /**
    * @brief Service to set ON/OFF the LED of the motors.
    * 
    * @param req p_req: Motor name and led(ON/OFF) vectors.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_led(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res);

    /**
    * @brief Service to create a homing trajectory setting only the time to reach the homing position..
    * 
    * @param req p_req: Slave Name, Trajectory Name, time vector.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motor_homing_trajectory(ec_srvs::SetMotorHomingTrj::Request  &req,ec_srvs::SetMotorHomingTrj::Response &res);
    
    /**
    * @brief Service to create a pediodic trajectory
    * 
    * @param req p_req: Slave Name, Trajectory Name,Frequency, Amplitude, Theta and Seconds.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motor_period_trajectory(ec_srvs::SetMotorPeriodTrj::Request  &req,ec_srvs::SetMotorPeriodTrj::Response &res);
    
    /**
    * @brief ...
    * 
    * @param req p_req: Slave Name, Trajectory Name,Time and Position Vectors
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motor_smooth_trajectory(ec_srvs::SetMotorSmoothTrj::Request  &req,ec_srvs::SetMotorSmoothTrj::Response &res);
    
    
    /**
    * @brief Service to start the trajectory of the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool start_motors_trajectory(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to clear the trajectory of the motors.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool clear_motors_trajectory(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);

    
private:
    
    std::shared_ptr<ros::NodeHandle>  _node;

    std::map<std::string, Motor_Setup::Ptr> _motor_map_setup;
    std::map<std::string, int> _id_map_motors;
    std::vector<std::string> _internal_list_motors;
    
    EC_Client_CMD::Ptr _ec_client_cmd;
    std::map<std::string,EC_Client_PDO::Ptr>_ec_client_pdo_map;
    ec_msgs::MotorPDO _motors_pdo;
    
    bool _status_callback;
    bool _xbotcorepresence;
    
    XBot::ModelInterface::Ptr _model;
    std::shared_ptr<RobotViz> _robotviz;
    XBot::JointNameMap _q;
    
    
    ros::Publisher _pub_motor_pdo;
    
    ros::ServiceServer _servie_print_motors_info,
                    _service_stop_motors,
                    _service_start_motors_posmode,
                    _service_start_motors_velmode,
                    _service_start_motors_impmode,
                    _service_release_motors_brake,
                    _service_engage_motors_brake,
                    _service_get_motors_ampere_info,
                    _service_get_motors_limits_info,
                    _service_get_motors_gains_info,
                    _service_set_motors_max_ampere,
                    _service_set_motors_posgains,
                    _service_set_motors_velgains,
                    _service_set_motors_impgains,
                    _service_set_motors_limits,
                    _service_set_motors_position,
                    _service_set_motors_velocity,
                    _service_set_motors_torque,
                    _service_set_motors_amperage,
                    _service_set_motors_homing_position,
                    _service_set_motors_fan,
                    _service_set_motors_led,
                    _service_set_motor_homing_trajectory,
                    _service_set_motor_period_trajectory,
                    _service_set_motor_smooth_trajectory,
                    _service_start_motors_trajectory,
                    _service_clear_motors_trajectory;
    
};

#endif
