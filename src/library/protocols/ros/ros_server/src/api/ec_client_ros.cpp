#include "ec_client_ros.h"
#include "ec_msgs/SlaveInfo.h"

#include <boost/asio.hpp>

#define MAX_ATTEMPTS 5000

using namespace std;
using namespace iit::advr;

EC_Client_ROS::EC_Client_ROS(std::string zmq_uri,int timeout):
_zmq_uri(zmq_uri),
_timeout(timeout)
{
    _ros_queue.reset();
    _ros_queue = std::make_unique<ros::CallbackQueue>();

    _node.reset();
    _node = std::make_shared<ros::NodeHandle>("ec_client");
    
    _node->setCallbackQueue(_ros_queue.get());
    
    _status_callback=true;

    
    _service_stop_master                        =   _node->advertiseService("stop_master", &EC_Client_ROS::stop_master, this);
    _service_start_master                       =   _node->advertiseService("start_master", &EC_Client_ROS::start_master, this);
    _service_get_slaves_descritption            =   _node->advertiseService("get_slaves_description", &EC_Client_ROS::get_slaves_descritption, this);
    
    _xbotcore_sts_sub                           =   _node->subscribe("/xbotcore/status", 1, &EC_Client_ROS::xbotcoreStatusCallback, this);
    
    _actual_xbotcore_state=_past_xbotcore_state="Inactive";

    _xbotcorepresence = false;

    _slave_ros = std::make_shared<Slave_ROS>(_node);
    
    start_zmq_mechanism();

}

bool EC_Client_ROS::callAvailable()
{
    _ros_queue->callAvailable();
    
    return (_status_callback && _slave_ros->get_status_callback());
}

void EC_Client_ROS::start_zmq_mechanism()
{
    _status_callback=true;
    _slave_ros->set_status_callback(true);
    
    _ec_client_cmd.reset();
    _ec_client_cmd = std::make_shared<EC_Client_CMD>(_zmq_uri,_timeout);  
    
    _slave_ros->set_ec_client_cmd(_ec_client_cmd);
}

void EC_Client_ROS::xbotcoreStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  _actual_xbotcore_state=msg->data;
}


void EC_Client_ROS::xbotcore_presence_STM()
{
    if(_actual_xbotcore_state=="Running")
    {
        if(_xbotcore_sts_sub.getNumPublishers()>0)
        {
            if(_actual_xbotcore_state!=_past_xbotcore_state)
            {
                _xbotcorepresence=true;
                _slave_ros->set_xbotcorepresence(_xbotcorepresence);
                _past_xbotcore_state=_actual_xbotcore_state;
                _get_param_attempt=0;
            }
            if(_get_param_attempt<MAX_ATTEMPTS)
            {
                std::string joint_id_map="",low_level_config="";
                _node->getParam("/xbotcore/low_level_config",low_level_config);
                _node->getParam("/xbotcore/joint_id_map",joint_id_map);
                
                if((joint_id_map!="")&&(low_level_config!=""))
                {
                    std::cout << "FOUND joint_id_map and low_level_config params" << std::endl;

                    if(!_slave_ros->set_internal_slaves(joint_id_map,low_level_config))
                    {
                        std::cout << "Error on XBotCore Paramaters " << std::endl;
                        std::cout << "Please restart XBotCore with the right ones.... " << std::endl;
                        _get_param_attempt=MAX_ATTEMPTS;
                    }
                    else
                    {
                        _get_param_attempt=MAX_ATTEMPTS;
                    }
                }
                else
                {
                    _get_param_attempt++; 
                    if(_get_param_attempt==MAX_ATTEMPTS-1)
                    {
                        std::cout << "XBotCore Paramaters not FOUND! " << std::endl;
                    }
                }
            }
            
        }
        else
        {
            _actual_xbotcore_state="Inactive"; //reset actual_xbotcore_state
        }
        
    }
    else
    {
        if(_actual_xbotcore_state!=_past_xbotcore_state)
        {
            _xbotcorepresence=false;
            _slave_ros->set_xbotcorepresence(_xbotcorepresence);
            _past_xbotcore_state=_actual_xbotcore_state;
            _get_param_attempt=0;
        }
    }
}


void EC_Client_ROS::pub_ec_client_info()
{
    _slave_ros->publish_slaves_info();
}

bool EC_Client_ROS::stop_master(ec_srvs::GetSlaveInfo::Request  &req,ec_srvs::GetSlaveInfo::Response &res)
{
    if(!_xbotcorepresence)
    {
        _status_callback=true;
        
        std::map<std::string,std::string> args;
        std::string msg;
        
        _ec_client_cmd->Ecat_Master_cmd(Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_STOP_MASTER,
                                        args,
                                        msg);
                                        
        EC_Client_CMD_Fault fault(_ec_client_cmd->get_fault());
        
        
        
        if(fault.get_type()==EC_CLIENT_CMD_STATUS::OK)
        {
            res.cmd_info.status="Master stopped successfully";
        }
        else
        {
            res.cmd_info.status="FAULT";
            if(fault.get_type()==EC_CLIENT_CMD_STATUS::TIMEOUT)
                _status_callback=false;
        }
        
        res.cmd_info.fault_cmd= fault.get_zmq_cmd();
        res.cmd_info.fault_info = fault.get_info();
        res.cmd_info.fault_recovery = fault.get_recovery_info();
        res.cmd_info.cmd="EtherCAT Master stop command";

        
        return true;
    }
    else
    {
        return false;
    }
}

bool EC_Client_ROS::start_master(ec_srvs::GetSlaveInfoStartMaster::Request  &req,ec_srvs::GetSlaveInfoStartMaster::Response &res)
{
    if(!_xbotcorepresence)
    {

        _status_callback=true;
        
        std::map<std::string,std::string> args;
        std::string msg;
        
        args["app_mode"]=req.mode;
        
        _ec_client_cmd->Ecat_Master_cmd(Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_START_MASTER,
                                        args,
                                        msg);

        EC_Client_CMD_Fault fault(_ec_client_cmd->get_fault());
        
        if(fault.get_type()==EC_CLIENT_CMD_STATUS::OK)
        {
            res.cmd_info.status="Master started successfully";
        }
        else
        {
            res.cmd_info.status="FAULT";
            if(fault.get_type()==EC_CLIENT_CMD_STATUS::TIMEOUT)
                _status_callback=false;
        }
        
        res.cmd_info.fault_cmd= fault.get_zmq_cmd();
        res.cmd_info.fault_info = fault.get_info();
        res.cmd_info.fault_recovery = fault.get_recovery_info();
        res.cmd_info.cmd="EtherCAT Master start command";

        
        return true;
    }
    else
    {
        return false;
    }
    
}

bool EC_Client_ROS::get_slaves_descritption(ec_srvs::GetSlaveInfo::Request  &req,ec_srvs::GetSlaveInfo::Response &res)
{
    string get_slaves_description="";
    
     _status_callback=true;
    
    std::map<std::string,std::string> args;
    
    _ec_client_cmd->Ecat_Master_cmd(Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR,
                                    args,
                                    get_slaves_description);
    
    EC_Client_CMD_Fault fault(_ec_client_cmd->get_fault());
    
    
    if(fault.get_type()==EC_CLIENT_CMD_STATUS::OK)
    {
        res.cmd_info.status="Received slaves description";
        // load slave descr as yaml
        YAML::Node slaves_info = YAML::Load(get_slaves_description);
        _ec_info_map = slaves_info.as<std::map<int, std::map<std::string, int>>>();
        
        ec_msgs::SlaveInfo slave_info_msg;
        
        for(auto first_item : _ec_info_map)
        {
            slave_info_msg.slave_id = first_item.first;
            auto slave_info = first_item.second;

            for(auto second_item : slave_info)
            {
                slave_info_msg.slave_info_name.push_back(second_item.first);
                slave_info_msg.slave_info_value.push_back(second_item.second);
            }
        }
        res.slaves_info.push_back(slave_info_msg);
        _slave_ros->set_ec_map_info(_ec_info_map);
        
        std::string slave_map_cfg=_slave_ros->get_slave_map_cfg();
        std::string low_level_config=_slave_ros->get_low_level_cfg();
        if((slave_map_cfg!="")&&(low_level_config!=""))
        {
            _slave_ros->set_internal_slaves(slave_map_cfg,low_level_config);
        }
        
    }
    else
    {
        res.cmd_info.status="FAULT";
        if(fault.get_type()==EC_CLIENT_CMD_STATUS::TIMEOUT)
            _status_callback=false;
    }
    
    res.cmd_info.fault_cmd= fault.get_zmq_cmd();
    res.cmd_info.fault_info = fault.get_info();
    res.cmd_info.fault_recovery = fault.get_recovery_info();
    res.cmd_info.cmd="EtherCAT Master Get Slave Description command";
    res.cmd_info.msg=get_slaves_description;
    
    
    return true;
}
