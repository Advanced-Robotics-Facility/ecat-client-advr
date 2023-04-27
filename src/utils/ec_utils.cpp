#include "ec_utils.h"

#include <iostream>

using namespace std;

EcUtils::EcUtils(const YAML::Node & ec_cfg)
{   
    
    // Read the EC Client configuration.
    
    //****** Protocol **************//
    if(!ec_cfg["network"]["protocol"])
        _ec_cfg.protocol="";
    else
        _ec_cfg.host_name=ec_cfg["network"]["protocol"].as<std::string>();
    
    if(!ec_cfg["network"]["hostname"])
        _ec_cfg.host_name="localhost";
    else
        _ec_cfg.host_name=ec_cfg["network"]["hostname"].as<std::string>();

    if(!ec_cfg["network"]["port"])
        _ec_cfg.host_port=0;
    else
        _ec_cfg.host_port=ec_cfg["network"]["port"].as<std::uint32_t>();
    
    if(!ec_cfg["period_ms"])
        _ec_cfg.period_ms=100;
    else
        _ec_cfg.period_ms=ec_cfg["period_ms"].as<int>();
    
    //****** Trajectory **************//
    if(ec_cfg["control"])
    {
        if(!ec_cfg["control"]["control_mode"])
            _control_mode="position";
        else
            _control_mode=ec_cfg["control"]["control_mode"].as<std::string>();
        
        if(_control_mode=="position")
        {
            _ec_cfg.control_mode_type=0x3B;
        }
        else if(_control_mode=="velocity")
        {
            _ec_cfg.control_mode_type=0x71;
        }
        else if(_control_mode=="impedance")
        {
            _ec_cfg.control_mode_type=0xD4;
        }
        else
        {
            throw std::runtime_error("Control mode not recognized, only the position, velocity or impedance mode is valid ");
        }
        
        if(!ec_cfg["control"]["gains"])
            _ec_cfg.gains.clear();
        else
            _ec_cfg.gains=ec_cfg["control"]["gains"].as<std::vector<float>>();
        
        if(_ec_cfg.gains.size()!=5)
        {
            throw std::runtime_error("Error invalid dimension of impedance gains. It has to be equal to five! ");
        }
        
        if(ec_cfg["control"]["homing_position"])
            _ec_cfg.homing_position=ec_cfg["control"]["homing_position"].as<map<double,double>>();
        
        if(_ec_cfg.homing_position.empty())
        {
            throw std::runtime_error("Homing position vector is empty, please setup the Homing position into the configuration file");
        }
        
        if(!ec_cfg["control"]["homing_time_sec"])
            _ec_cfg.homing_time_sec=5; // 5 second
        else
            _ec_cfg.homing_time_sec=ec_cfg["control"]["homing_time_sec"].as<int>();
        
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.homing_time_sec*1000))
        {
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(ec_cfg["control"]["trajectory"])
            _ec_cfg.trajectory=ec_cfg["control"]["trajectory"].as<map<double,double>>();
        
        if(_ec_cfg.trajectory.empty())
        {
            throw std::runtime_error("Trajectoy vector is empty, please setup the Homing position into the configuration file");
        }
        
        if(_ec_cfg.homing_position.size() != _ec_cfg.trajectory.size())
        {
        throw std::runtime_error("Size of homing position vector and trajectory vector is different, please setup the same dimension"); 
        }
        
        if(!ec_cfg["control"]["trajectory_time_sec"])
            _ec_cfg.trajectory_time_sec=5; // 5 second
        else
            _ec_cfg.trajectory_time_sec=ec_cfg["control"]["trajectory_time_sec"].as<int>();
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.trajectory_time_sec*1000))
        {
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(!ec_cfg["control"]["repeat_trj"])
            _ec_cfg.repeat_trj=1;
        else
            _ec_cfg.repeat_trj=ec_cfg["control"]["repeat_trj"].as<int>();
        
        if(!ec_cfg["control"]["slave_id_led"])
            _ec_cfg.slave_id_led.clear();
        else
            _ec_cfg.slave_id_led=ec_cfg["control"]["slave_id_led"].as<std::vector<int>>();
    }

};

EcUtils::EC_CONFIG EcUtils::get_ec_cfg()
{
    return _ec_cfg;
};



EcUtils::~EcUtils()
{
};

