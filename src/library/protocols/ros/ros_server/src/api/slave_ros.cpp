#include "slave_ros.h"
#include "slave_cmd.h"

#include <esc_info.h>

#include <boost/asio.hpp>

using namespace std;

Slave_ROS::Slave_ROS(std::shared_ptr<ros::NodeHandle> node):
_node(node)
{

    _status_callback=true;

    
    _service_get_slaves_firmware                =   _node->advertiseService("get_slaves_firmware", &Slave_ROS::get_slaves_firmware, this);
    _service_get_slaves_sdo_info_by_name        =   _node->advertiseService("get_slaves_sdo_info_by_name", &Slave_ROS::get_slaves_sdo_info_by_name, this);
    _service_get_slaves_sdo_info_by_objd        =   _node->advertiseService("get_slaves_sdo_info_by_objd", &Slave_ROS::get_slaves_sdo_info_by_objd, this);
    _service_set_slaves_firmware                =   _node->advertiseService("set_slaves_firmware", &Slave_ROS::set_slaves_firmware, this);
    
    _service_save_slaves_params_to_flash        =   _node->advertiseService("save_slaves_params_to_flash", &Slave_ROS::save_slaves_params_to_flash, this);
    _service_load_slaves_params_from_flash      =   _node->advertiseService("load_slaves_params_from_flash", &Slave_ROS::load_slaves_params_from_flash, this);
    _service_load_slaves_default_params         =   _node->advertiseService("load_slaves_default_params", &Slave_ROS::load_slaves_default_params, this);
    
    _service_set_slaves_sdo_cmd                 =   _node->advertiseService("set_slaves_sdo_cmd", &Slave_ROS::set_slaves_sdo_cmd, this);

    
    _service_set_slaves                         =   _node->advertiseService("set_slaves", &Slave_ROS::set_slaves, this);

    _service_start_pub_pdo                      =   _node->advertiseService("start_pub_pdo", &Slave_ROS::start_pub_pdo, this);
    _service_stop_pub_pdo                       =   _node->advertiseService("stop_pub_pdo", &Slave_ROS::stop_pub_pdo, this);
    
    _xbotcorepresence=false;
    
    _motor_ros = std::make_shared<Motor_ROS>(_node);
    _pow_ros   = std::make_shared<POW_ROS>(_node);
    _imu_ros   = std::make_shared<IMU_ROS>(_node);
    _ft_ros    = std::make_shared<FT_ROS>(_node);
    
    _start_pub_pdo=false;
}

void Slave_ROS::set_ec_client_cmd(EC_Client_CMD::Ptr ec_client_cmd)
{   
    _ec_client_cmd.reset();
    _ec_client_cmd=ec_client_cmd;
    
    _motor_ros->set_ec_client_cmd(ec_client_cmd);
    
}

void Slave_ROS::set_status_callback(bool status_callback)
{
    _status_callback=status_callback;
    _motor_ros->set_status_callback(status_callback);
}
void Slave_ROS::set_ec_map_info(std::map<int, SlaveInfoMap> ec_info_map)
{
    if(!_ec_info_map.empty())
    {
        _ec_info_map.clear();
    }
    _ec_info_map=ec_info_map;
}

bool Slave_ROS::get_status_callback()
{
    return _status_callback && _motor_ros->get_status_callback();
}

void Slave_ROS::set_xbotcorepresence(bool xbotcorepresence)
{
    _xbotcorepresence=xbotcorepresence;
    _motor_ros->set_xbotcorepresence(xbotcorepresence);
}


bool Slave_ROS::start_pub_pdo(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res)
{
    if(_internal_list_slaves.empty())
    {
        _start_pub_pdo=false;
        return false;
    }
   _start_pub_pdo=true;

   return true;

}
bool Slave_ROS::stop_pub_pdo(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res)
{
   _start_pub_pdo=false;

   return true;
}

void Slave_ROS::publish_slaves_info()
{
    if(_start_pub_pdo)
    {
        if((!_motor_ros->pub_motors_pdo())||
           (!_pow_ros->pub_pow_pdo())||  
           (!_imu_ros->pub_imu_pdo())||
           (!_ft_ros->pub_ft_pdo()))
        {
            _start_pub_pdo=false;
        }

//         _ft_ros->pub_motors_pdo();
    }
    
    _motor_ros->pub_robot();
}

void Slave_ROS::clear_slaves_structures()
{
    
    _motor_map_setup.clear();

    _ec_client_pdo_map.clear();

    _map_id_slaves.clear();

    _internal_list_slaves.clear();

    _list_pow_slaves.clear();
    
    _list_imu_slaves.clear();
    
    _list_ft_slaves.clear();

    _q_home.clear();
    
}

std::string Slave_ROS::get_slave_map_cfg()
{
  return _slave_map_cfg;  
}
    
std::string Slave_ROS::get_low_level_cfg()
{
   return _low_level_cfg;
}

bool Slave_ROS::set_internal_slaves(std::string slave_map_cfg,std::string low_level_cfg)
{
    clear_slaves_structures();
     _start_pub_pdo=false;
     
     if(_model)
         _model->getRobotState("home", _q_home);
     
    try{
        auto joint_map_path_yaml = YAML::Load(slave_map_cfg);
        auto joint_map_yaml = joint_map_path_yaml["joint_map"];
        try{
            auto low_level_cfg_yaml= YAML::Load(low_level_cfg);
            if(!low_level_cfg_yaml["zmq_setup"]["timeout"])
                _timeout=1000; // 1s of timeout;
            else
                _timeout=low_level_cfg_yaml["zmq_setup"]["timeout"].as<int>();
    
            _ec_client_cmd->set_zmq_timeout(_timeout);
            
            
            for(YAML::const_iterator it=joint_map_yaml.begin();it!=joint_map_yaml.end();++it)
            {
                int slave_id=it->first.as<int>();
                string slave_name=it->second.as<std::string>();
                
                int board_type=iit::ecat::NO_TYPE;
                
                _map_id_slaves[slave_name]=slave_id;
                _internal_list_slaves.push_back(slave_name);
                
                if(!_ec_info_map.empty())
                {
                    if(_ec_info_map.count(slave_id)>0)
                    {
                        board_type=_ec_info_map.at(slave_id).at("esc_type");
                    }
                    else
                    {
                        _map_id_slaves.erase(slave_name);
                        _internal_list_slaves.pop_back();
                    }
                }

                if(_map_id_slaves.count(slave_name)>0)
                {
                    auto pdo_port=9000+slave_id;
            
                    string zmq_uri_pdo="tcp://"+boost::asio::ip::host_name()+":"+std::to_string(pdo_port);
                    //std::cout << zmq_uri_pdo << std::endl;
                    
                    EC_Client_PDO::Ptr ec_client_pdo=std::make_shared<EC_Client_PDO>(zmq_uri_pdo); 
                    pair<map<string, EC_Client_PDO::Ptr>::iterator,bool> ret=_ec_client_pdo_map.insert(pair<string, EC_Client_PDO::Ptr>(slave_name,ec_client_pdo));
                    
                    if(!ret.second)
                    {
                        cout << "Error: ID "<<slave_name <<" already setup"<<endl;
                        return false;
                    }
                    
                    if((board_type==iit::ecat::CENT_AC)||
                        (board_type==iit::ecat::LO_PWR_DC_MC)||
                        (board_type==iit::ecat::NO_TYPE))
                    {
                        if(!low_level_cfg_yaml[slave_name])
                        {
                            cout << "Warning: Joint "<< slave_name <<" doesn't exist in the low level file" << endl;
                        }
                        else
                        {
                            Motor_Setup::Ptr motor_setup = std::make_shared<Motor_Setup>(low_level_cfg_yaml[slave_name],slave_id,slave_name);
                            pair<map<string, Motor_Setup::Ptr>::iterator,bool> ret=_motor_map_setup.insert(pair<string, Motor_Setup::Ptr>(slave_name,motor_setup));
                            if(!ret.second)
                            {
                                cout << "Error: ID "<<slave_name <<" already setup"<<endl;
                                return false;
                            }
                            
                            if(_q_home.count(slave_name)>0)
                                motor_setup->set_homing_position(_q_home.at(slave_name));
                        }
                    }
                    else if(board_type==iit::ecat::Board_type::POW_F28M36_BOARD)
                    {
                        _list_pow_slaves.push_back(slave_name);
                    }
                    else if(board_type==iit::ecat::Board_type::IMU_ANY)
                    {
                        _list_imu_slaves.push_back(slave_name);
                    }
                    else if(board_type==iit::ecat::Board_type::FT6)
                    {
                        _list_ft_slaves.push_back(slave_name);
                    }
                }
            }
            
            _slave_map_cfg=slave_map_cfg;
            _low_level_cfg=low_level_cfg;
            
            _motor_ros->set_motor_map_setup(_motor_map_setup);
            _motor_ros->set_ec_client_pdo_map(_ec_client_pdo_map);
            
            _motor_ros->clear_joint_map();
            _motor_ros->clear_motors_pdo_message();
            
            _pow_ros->set_internal_list_pows(_list_pow_slaves);
            _pow_ros->set_ec_client_pdo_map(_ec_client_pdo_map);
            _pow_ros->clear_pow_pdo_message();


            _imu_ros->set_internal_list_imus(_list_imu_slaves);
            _imu_ros->set_ec_client_pdo_map(_ec_client_pdo_map);
            _imu_ros->clear_imu_pdo_message(); 
            
            _ft_ros->set_internal_list_fts(_list_ft_slaves);
            _ft_ros->set_ec_client_pdo_map(_ec_client_pdo_map);
            _ft_ros->clear_ft_pdo_message();      
            

            
        }catch(std::exception &ex){
            cout << "Error on low level config file" << endl;
            cout << ex.what() << endl;
            return false;   
        }
        
    }catch(std::exception &ex){
        cout << "Error on slave_map config file" << endl;
        cout << ex.what() << endl;
        return false;
    }
    
        //start_zmq_mechanism();

    return true;
}


void Slave_ROS::set_internal_model_interface(std::vector<std::string> robot_info,std::string joint_map)
{
  _model.reset();
  
  if((robot_info[0]!="")&&(robot_info[1]!="")&&(robot_info[2]!="")&&(robot_info[3]!=""))
  {
    XBot::ConfigOptions config;
    config.set_urdf(robot_info[0]);
    config.set_srdf(robot_info[1]);
    config.generate_jidmap();
    //config.set_jidmap(joint_map); @TO DO
    config.set_parameter("model_type",robot_info[2]);
    config.set_parameter("is_model_floating_base",false);
    if(robot_info[3]=="true")
    {
        config.set_parameter("is_model_floating_base",true);
    }
    
    _model=XBot::ModelInterface::getModel(config);
    RobotViz::color rgba = Eigen::Vector4d(0.0, 1.0, 0.0, 0.5);
    std::string robot_name_space=_node->getNamespace()+"/robot";
    _robotviz=std::make_shared<RobotViz>(_model,robot_name_space,*_node,rgba);
    
    _motor_ros->set_model_interface(_model);
    _motor_ros->set_robotviz(_robotviz);
    
  }
}

bool Slave_ROS::set_slaves(ec_srvs::SetMotorConfigFile::Request  &req,ec_srvs::SetMotorConfigFile::Response &res)
{
    string low_level_cfg(req.low_level_cfg),slave_map_cfg(req.slave_map_cfg);
    
    Slave_ROS::set_internal_model_interface(req.robot_info,slave_map_cfg);
    bool ret= Slave_ROS::set_internal_slaves(slave_map_cfg,low_level_cfg);
    
    return ret;
}


bool Slave_ROS::get_slaves_firmware(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    std::vector<std::string> rd_sdo;
    std::map<std::string ,std::string> wr_sdo;
    
    std::string cmd="get_slaves_firmware";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        rd_sdo.push_back("m3_fw_ver");
        rd_sdo.push_back("c28_fw_ver");
        
        slave_sdo_cmd(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}

bool Slave_ROS::get_slaves_sdo_info_by_name(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    
    std::string cmd="get_slaves_sdo_info_by_name";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        slave_sdo_info(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,iit::advr::Slave_SDO_info_Type::Slave_SDO_info_Type_SDO_NAME,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}

bool Slave_ROS::get_slaves_sdo_info_by_objd(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    
    std::string cmd="get_slaves_sdo_info_by_objd";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        slave_sdo_info(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,iit::advr::Slave_SDO_info_Type::Slave_SDO_info_Type_SDO_OBJD,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}


bool Slave_ROS::save_slaves_params_to_flash(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    
    std::string cmd="flash_slaves";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        flash_cmd(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,iit::advr::Flash_cmd_Type::Flash_cmd_Type_SAVE_PARAMS_TO_FLASH,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}

bool Slave_ROS::load_slaves_params_from_flash(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    
    std::string cmd="load_slaves_params_from_flash";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        flash_cmd(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,iit::advr::Flash_cmd_Type::Flash_cmd_Type_LOAD_PARAMS_FROM_FLASH,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}

bool Slave_ROS::load_slaves_default_params(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    
    std::string cmd="load_slaves_params_from_flash";
    
    if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
    {
        flash_cmd(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,iit::advr::Flash_cmd_Type::Flash_cmd_Type_LOAD_DEFAULT_PARAMS,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}

bool Slave_ROS::set_slaves_sdo_cmd(ec_srvs::SetSlaveSdo::Request& req, ec_srvs::SetSlaveSdo::Response& res)
{
    // what is the purpose of this flag?
    _status_callback = true;

    // select requested slaves
    std::vector<std::string> slaves_selected;
    select_slaves(_internal_list_slaves, req.slave_name, slaves_selected, res.cmd_status);
    
    // sdo values
    std::vector<float> sdo_value;

    // if "all" or "" was specified, we must apply the first (and only)
    // sdo value to all slaves
    if(req.slave_name == std::vector<std::string>{"all"} || 
        req.slave_name == std::vector<std::string>{""})
    {
        sdo_value.assign(slaves_selected.size(), req.sdo_value[0]);
    }
    else
    {
        sdo_value = req.sdo_value;
    }

    // empty rd sdo (we want to write here, not read)
    std::vector<std::string> rd_sdo;
    
    // name of the command
    std::string cmd = req.sdo_name;
    
    for(int i = 0; i < slaves_selected.size(); i++)
    {        
        // slave name and id
        auto slave_name = slaves_selected[i];
        int slave_id = _map_id_slaves.at(slave_name);

        // sdo to write
        std::map<std::string, std::string> wr_sdo;
        wr_sdo[req.sdo_name] = std::to_string(sdo_value[i]);
            
        // call set sdo service
        std::string msg;
        _ec_client_cmd->Slave_SDO_cmd(slave_id, rd_sdo, wr_sdo, msg);

        // check reply
        EC_Client_CMD_Fault fault(_ec_client_cmd->get_fault());

        if(fault.get_type() == EC_CLIENT_CMD_STATUS::OK)
        {
            std::cout << "ok     " << cmd << " on slave '" << slave_name << "'" << std::endl;
        }
        else
        {
            std::cout << "failed " << cmd << " on slave '" << slave_name << "'" << std::endl;
        }
    }
        
    return true;
};

bool Slave_ROS::set_slaves_firmware(ec_srvs::SetSlavesFirmware::Request  &req,ec_srvs::SetSlavesFirmware::Response &res)
{
    if(!_xbotcorepresence)
    {
        _status_callback=true;
    
        std::vector<std::string> slaves_selected;
        
        std::string cmd="set_slaves_firmware";
        
        if(select_slaves(_internal_list_slaves,req.slave_name,slaves_selected,res.cmd_status))
        {
            std::vector<std::string> foe_info;
            foe_info.push_back(req.m3_file);
            foe_info.push_back(req.m3_password);
            foe_info.push_back(req.c28_file);
            foe_info.push_back(req.c28_password);
            
            if(!foe_master(slaves_selected,_map_id_slaves,_ec_client_cmd,cmd,foe_info,res.slaves_info,res.cmd_status,_status_callback))
            {
                return false;
            }
        }
        
        return true;
    }
    else
    {
        return false;
    }
};
