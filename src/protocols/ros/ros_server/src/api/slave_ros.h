#ifndef __SLAVE_ROS__
#define __SLAVE_ROS__

#include "motor_setup.h"
#include "ec_client_cmd.h"
#include "ec_client_pdo.h"
#include "motor_ros.h"
#include "pow_ros.h"
#include "imu_ros.h"
#include "ft_ros.h"
// 
#include "ros/ros.h"

#include "ec_msgs/SlaveCmdInfo.h"
#include "ec_msgs/CmdStatus.h"
#include "ec_msgs/MotorPDO.h"


#include "ec_srvs/SelectSlave.h"
#include "ec_srvs/SetSlaveSdo.h"
#include "ec_srvs/SetMotorConfigFile.h"

#include "ec_srvs/SetSlavesFirmware.h"
#include "rviz/robot_viz.h"
#include <std_srvs/Empty.h>


class Slave_ROS
{
public:
    
    typedef std::shared_ptr<Slave_ROS> Ptr;
    typedef std::map<std::string, int> SlaveInfoMap;

 
    /**
    * @brief Constructor of Slave ROS Class
    * It needs of shared pointer to the ROS Node Handle for the topics/services.
    * 
    * @param node p_node: ROS Node Handle.
    */
    Slave_ROS(std::shared_ptr<ros::NodeHandle> node);
    
    /**
    * @brief Destructor of Slave ROS Class
    * 
    */
    ~Slave_ROS(){
    };
    
    /**
    * @brief Service to enable the publishing of Slaves PDOs. 
    * 
    * @param req p_req: NONE
    * @param res p_res: NONE
    * @return bool
    */
    bool start_pub_pdo(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);
    
    /**
    * @brief Service to disable the publishing of Slaves PDOs.
    * 
    * @param req p_req: NONE
    * @param res p_res: NONE
    * @return bool
    */
    bool stop_pub_pdo(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);
    
    /**
    * @brief Service to get the slaves firmware information. 
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool get_slaves_firmware(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
        /**
    * @brief Service to set a generic SDO command to the slaves
    * 
    * @param req p_req: Motor name and sdo name / values
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_motors_sdo_cmd(ec_srvs::SetSlaveSdo::Request  &req,ec_srvs::SetSlaveSdo::Response &res);
    
    /**
    * @brief Service to get by NAME the slaves SDO information .
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool get_slaves_sdo_info_by_name(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to get by OBJECT DICTIONARY the slaves SDO information .
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool get_slaves_sdo_info_by_objd(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    

    /**
    * @brief Service to set the SDO on the slaves.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool set_slaves_sdo_cmd(ec_srvs::SetSlaveSdo::Request& req, ec_srvs::SetSlaveSdo::Response& res);
    
    /**
    * @brief Service to save the parameters into the flash of the slaves.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool save_slaves_params_to_flash(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to load the parameters from the flash of the slaves.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool load_slaves_params_from_flash(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);

    /**
    * @brief Service to load default parameters from the flash of the slaves.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback. 
    * @return bool
    */
    bool load_slaves_default_params(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res);
    
    /**
    * @brief Service to set the firmware of the slaves.
    * 
    * @param req p_req: Slaves name vector. Note: it's possible to send an empty vector or insert the ALL keyword to select ALL slaves.
    * Firmware information: c28 and m28 files, c28 and m3 passwords.
    * @param res p_res: The response has two messages, one related on the command information for every slave (FAULT, CMD Type, Message, etc...)
    * the other one related on command status for all slaves, which slaves have had a success or unsuccess command feedback.
    * @return bool
    */
    bool set_slaves_firmware(ec_srvs::SetSlavesFirmware::Request  &req,ec_srvs::SetSlavesFirmware::Response &res);

    /**
    * @brief Service to set all slaves, especially for the motors.
    * 
    * @param req p_req: Low Level Configuration File, Slave Map File, string vector of robot information having:
    * 1) URDF
    * 2) SRDF.
    * 3) model_type (RBDL...).
    * 4) is_model_floating_base (true/false).
    * @param res p_res: NONE
    * @return bool
    */
    bool set_slaves(ec_srvs::SetMotorConfigFile::Request  &req,ec_srvs::SetMotorConfigFile::Response &res);
    
    /**
    * @brief This method is responsible to set the slaves using the low level configuration and the slave map.
    * 
    * @param slave_map_cfg p_slave_map_cfg: Slave map of id and slave name.
    * @param low_level_cfg p_low_level_cfg: low level configuration file of the motors, IMUs, FTs, POWs and etc...
    * @return bool
    */
    bool set_internal_slaves(std::string slave_map_cfg,
                             std::string low_level_cfg);
    
    /**
    * @brief This method is responsible to buld a model interface using the robot information (URDF, SRDF, model_type and is_model_floating_base) and create a RobotViz object to publish the robot pose.
    * 
    * @param robot_info p_robot_info: URDF, SRDF, model_type and is_model_floating_base information.
    * @param joint_map p_joint_map: Joint map of id and slave name.
    */
    void set_internal_model_interface(std::vector<std::string> robot_info,std::string joint_map);
    
    /**
    * @brief Method to clear the slaves structures ( motor map, ecat pdo map, etc ...).
    * 
    */
    void clear_slaves_structures();

    /**
    * @brief Method that returns the slave map.
    * 
    * @return std::__cxx11::string
    */
    std::string get_slave_map_cfg();
    
    /**
    * @brief Method that returns the low level configuration file.
    * 
    * @return std::__cxx11::string
    */
    std::string get_low_level_cfg();
    
    /**
    * @brief This method allows to set the XbotCore Presence to forbid the services.
    * 
    * @param xbotcorepresence p_xbotcorepresence: set boolean value for the XBotCore presence.
    */
    void set_xbotcorepresence(bool xbotcorepresence);
   
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
    * @brief Method to set  map of slave information , slave_id, esc_type etc...
    * 
    * @param ec_info_map p_ec_info_map:...
    */
    void set_ec_map_info(std::map<int, SlaveInfoMap> ec_info_map);
    

    /**
    * @brief Method that return the status of the callbacks from the slaves.
    * 
    * @return bool
    */
    bool get_status_callback();
    
    /**
    * @brief Method to publish the slaves information.
    * 
    */
    void publish_slaves_info();
    
private:

    std::shared_ptr<ros::NodeHandle>  _node;
    EC_Client_CMD::Ptr _ec_client_cmd;

    
    ros::ServiceServer _service_get_slaves_firmware,
                       _service_get_slaves_sdo_info_by_name,
                       _service_get_slaves_sdo_info_by_objd,
                       _service_set_slaves_firmware,
                       _service_save_slaves_params_to_flash,
                       _service_load_slaves_params_from_flash,
                       _service_load_slaves_default_params,
                       _servie_print_slaves_info,
                       _service_set_slaves,
                       _service_set_slaves_sdo_cmd;
                       
    ros::ServiceServer _service_start_pub_pdo,
                       _service_stop_pub_pdo;
    
    
    
    std::map<std::string, Motor_Setup::Ptr> _motor_map_setup;
    
    std::map<std::string, int> _map_id_slaves;
    std::vector<std::string> _internal_list_slaves;
    std::vector<std::string> _list_pow_slaves;
    std::vector<std::string> _list_imu_slaves;
    std::vector<std::string> _list_ft_slaves;
    
    std::map<std::string,EC_Client_PDO::Ptr>_ec_client_pdo_map;
    
    bool _status_callback;
    bool _xbotcorepresence;
    bool _start_pub_pdo;
    
    XBot::ModelInterface::Ptr _model;
    std::shared_ptr<RobotViz> _robotviz;

    int _timeout;
    
    Motor_ROS::Ptr _motor_ros;
    POW_ROS::Ptr _pow_ros;
    IMU_ROS::Ptr _imu_ros;
    FT_ROS::Ptr _ft_ros;
    
    std::string _slave_map_cfg;
    std::string _low_level_cfg;
    
    XBot::JointNameMap _q_home;
    
    std::map<int, SlaveInfoMap> _ec_info_map;
    
};

#endif
