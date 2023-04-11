#include "ec_utils.h"

#include <iostream>

using namespace std;

EC_Client_Utils::EC_Client_Utils(const YAML::Node & ec_client_cfg)
{   
    _ec_config.host_name_s="";
    
    // DEFAULT POSITION and 100ms PERIOD
    _control_mode="position";
    _ec_config.UDP_period_ms=100;    
    
    _ec_config.homing_time_sec=_ec_config.trajectory_time_sec=5; // 5second
    _ec_config.repeat_trj=1; // ON TIME
    
    // Read the EC Client configuration.
    if(!ec_client_cfg["network"]["hostname"])
        _ec_config.host_name_s="localhost";
    else
        _ec_config.host_name_s=ec_client_cfg["network"]["hostname"].as<std::string>();

    if(!ec_client_cfg["network"]["port"])
        _ec_config.host_port=50000;
    else
        _ec_config.host_port=ec_client_cfg["network"]["port"].as<std::uint32_t>();
    
    if(!ec_client_cfg["control_mode"])
        _control_mode="position";
    else
        _control_mode=ec_client_cfg["control_mode"].as<std::string>();
    
    if(_control_mode=="position")
    {
        _ec_config.control_mode_type=0x3B;
    }
    else if(_control_mode=="velocity")
    {
        _ec_config.control_mode_type=0x71;
    }
    else if(_control_mode=="impedance")
    {
        _ec_config.control_mode_type=0xD4;
    }
    else
    {
        throw std::runtime_error("Control mode not recognized, only the position, velocity or impedance mode is valid ");
    }
    
    if(!ec_client_cfg["gains"])
        _ec_config.gains.clear();
    else
        _ec_config.gains=ec_client_cfg["gains"].as<std::vector<float>>();
    
    if(_ec_config.gains.size()!=5)
    {
        throw std::runtime_error("Error invalid dimension of impedance gains. It has to be equal to five! ");
    }
    
    if(!ec_client_cfg["UDP_period_ms"])
        _ec_config.UDP_period_ms=100;
    else
        _ec_config.UDP_period_ms=ec_client_cfg["UDP_period_ms"].as<int>();
    
    if(ec_client_cfg["homing_position"])
        _ec_config.homing_position=ec_client_cfg["homing_position"].as<map<double,double>>();
    
    if(_ec_config.homing_position.empty())
    {
        throw std::runtime_error("Homing position vector is empty, please setup the Homing position into the configuration file");
    }
    
    if(!ec_client_cfg["homing_time_sec"])
        _ec_config.homing_time_sec=5; // 5 second
    else
        _ec_config.homing_time_sec=ec_client_cfg["homing_time_sec"].as<int>();
    
    
    if(2*_ec_config.UDP_period_ms> (_ec_config.homing_time_sec*1000))
    {
        throw std::runtime_error("The time of homing time should be at least two times higher then the UDP_period_ms variable");
    }
    
    
    if(ec_client_cfg["trajectory"])
        _ec_config.trajectory=ec_client_cfg["trajectory"].as<map<double,double>>();
    
    if(_ec_config.trajectory.empty())
    {
        throw std::runtime_error("Trajectoy vector is empty, please setup the Homing position into the configuration file");
    }
    
    if(_ec_config.homing_position.size() != _ec_config.trajectory.size())
    {
       throw std::runtime_error("Size of homing position vector and trajectory vector is different, please setup the same dimension"); 
    }
    
    if(!ec_client_cfg["trajectory_time_sec"])
        _ec_config.trajectory_time_sec=5; // 5 second
    else
        _ec_config.trajectory_time_sec=ec_client_cfg["trajectory_time_sec"].as<int>();
    
    if(2*_ec_config.UDP_period_ms> (_ec_config.trajectory_time_sec*1000))
    {
        throw std::runtime_error("The time of homing time should be at least two times higher then the UDP_period_ms variable");
    }
    
    
    if(!ec_client_cfg["repeat_trj"])
        _ec_config.repeat_trj=1;
    else
        _ec_config.repeat_trj=ec_client_cfg["repeat_trj"].as<int>();
    
    if(!ec_client_cfg["slave_id_led"])
        _ec_config.slave_id_led.clear();
    else
        _ec_config.slave_id_led=ec_client_cfg["slave_id_led"].as<std::vector<int>>();
    
    // Read the EC Client configuration END.

};

EC_Client_Utils::EC_CONFIG EC_Client_Utils::get_ec_client_config()
{
    return _ec_config;
};



EC_Client_Utils::~EC_Client_Utils()
{
};

