#ifndef __EC_CLIENT_ROS__
#define __EC_CLIENT_ROS__

#include "motor_setup.h"
#include "ec_client_cmd.h"

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "std_msgs/String.h"

#include "ec_srvs/SelectSlave.h"
#include "ec_srvs/GetSlaveInfo.h"
#include "ec_srvs/GetSlaveInfoStartMaster.h"

#include "slave_ros.h"


class EC_Client_ROS
{
public:
 
    typedef std::shared_ptr<EC_Client_ROS> Ptr;
    /**
    * @brief Constructor of EC Client ROS Class
    * It needs of zmq uri (TCP/UDP) and of a timeout.
    * 
    * @param zmq_uri p_zmq_uri: ZMQ URI to create the communication.
    * @param timeout p_timeout: Timeout for the communication.
    */
    EC_Client_ROS(std::string zmq_uri,
                      int timeout);
    /**
    * @brief Destructor of EC Client ROS Class
    * 
    */
    
    ~EC_Client_ROS(){
    };
    
    
    /**
    * @brief This method is responsible to start ROS queue, to receive the services checking the services errors and ZMQ communicatione errors of EC CLIENT ROS and for all slaves.
    * 
    * @return bool
    */
    bool callAvailable();
    
    /**
    * @brief This method is responsible to star the ZMQ mechanism to send the commands and receive the reply.
    * 
    */
    void start_zmq_mechanism();


    /**
    * @brief This is the service responsible to stop the EtherCAT Master server.
    * 
    * @param req p_req: NONE
    * @param res p_res: The response has two messages, one related on the command information (FAULT, CMD Type, Message, etc...)
    * the other one related on slave information like the slave id.
    * @return bool
    */
    bool stop_master(ec_srvs::GetSlaveInfo::Request  &req,
                     ec_srvs::GetSlaveInfo::Response &res);
    
    /**
    * @brief This is the service responsible to start the EtherCAT Master server.
    * 
    * @param req p_req:NONE
    * @param res p_res:The response has two messages, one related on the command information (FAULT, CMD Type, Message, etc...)
    * the other one related on slave information like the slave id.
    * @return bool
    */
    bool start_master(ec_srvs::GetSlaveInfoStartMaster::Request  &req,
                      ec_srvs::GetSlaveInfoStartMaster::Response &res);
    /**
    * @brief This is the service responsible to get the slaves description from the EtherCAT Master server.
    * 
    * @param req p_req:NONE
    * @param res p_res:The response has two messages, one related on the command information (FAULT, CMD Type, Message, etc...)
    * the other one related on slave information like the slave id.
    * @return bool
    */
    
    bool get_slaves_descritption(ec_srvs::GetSlaveInfo::Request  &req,
                                 ec_srvs::GetSlaveInfo::Response &res);
    
    /**
    * @brief This method is responsible to publish the PDOs of all slaves and the robot pose if present.
    * 
    */
    void pub_ec_client_info();

    /**
    * @brief This is the callback used to check if XBotCore node is present and if it's in Running status.
    * 
    * @param msg p_msg: Message used to check the XBotCore status.
    */
    void xbotcoreStatusCallback(const std_msgs::String::ConstPtr& msg);
    
    /**
    * @brief This the state machine with two states (XBotCore present or not present), responsible to forbid some services of the ec_client ROS and for the slaves. 
    * 
    */
    void xbotcore_presence_STM();
    
private:

    std::unique_ptr<ros::CallbackQueue> _ros_queue;
    std::shared_ptr<ros::NodeHandle>  _node;
    ros::Subscriber _xbotcore_sts_sub;
    
    bool  _xbotcorepresence;
    
    ros::ServiceServer _service_stop_master,
                       _service_start_master,
                       _service_get_slaves_descritption;
                       
    
    EC_Client_CMD::Ptr _ec_client_cmd;
    Slave_ROS::Ptr _slave_ros;
    
    std::string _zmq_uri;
    int _timeout,_get_param_attempt;
    bool _status_callback;
    
    std::string _actual_xbotcore_state, _past_xbotcore_state;
    
    typedef std::map<std::string, int> SlaveInfoMap;
    std::map<int, SlaveInfoMap> _ec_info_map;
};

#endif
