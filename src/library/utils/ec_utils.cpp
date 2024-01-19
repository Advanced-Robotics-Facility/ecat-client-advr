#include "utils/ec_utils.h"

#include <iostream>
#include <chrono>
#include <libgen.h>

using namespace std;
using namespace std::chrono;

EcUtils::EcUtils(EC_CONFIG ec_cfg):
_ec_cfg(ec_cfg)
{
    
}

EcUtils::EcUtils()
{   
    _ec_client_cfg_file= getenv ("EC_CLIENT_CFG");
    
    if (_ec_client_cfg_file.c_str()==NULL)
    {
        throw std::runtime_error("EC Client configuration not found, setup the environment variable: EC_CLIENT_CFG ");
    }
    
    auto ec_cfg=YAML::LoadFile(_ec_client_cfg_file);
    
    // Read the EC Client configuration.
    
    //****** Protocol **************//
    if(!ec_cfg["network"]["protocol"])
        _ec_cfg.protocol="";
    else
        _ec_cfg.protocol=ec_cfg["network"]["protocol"].as<std::string>();

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
    
    if(!ec_cfg["logging"])
        _ec_cfg.logging=false;
    else
        _ec_cfg.logging=ec_cfg["logging"].as<bool>();
    
    
    //****** Trajectory **************//
    if(ec_cfg["control"])
    {
        std::string motor_cfg_path="";
        if(ec_cfg["control"]["joint_config_path"])
        {
            motor_cfg_path=ec_cfg["control"]["joint_config_path"].as<std::string>();
        }
        
        std::string joint_map_path="";
        if(ec_cfg["control"]["joint_map_path"])
        {
            joint_map_path=ec_cfg["control"]["joint_map_path"].as<std::string>();
        }
        
        if(joint_map_path!="")
        {
            auto joint_map_file = YAML::LoadFile(joint_map_path);
            if(motor_cfg_path!="")
            {
                auto motor_cfg_file = YAML::LoadFile(motor_cfg_path);
                _ec_cfg.motor_config_map= get_motor_config_map(motor_cfg_file,joint_map_file);
            }
            else
            {
                throw std::runtime_error("Wrong joint configuration file!");
            }
        }
        else
        {
            throw std::runtime_error("Wrong joint map file!");
        }
        
        if(ec_cfg["control"]["homing_position"])
        {
           auto homing_position = ec_cfg["control"]["homing_position"];
           for(YAML::const_iterator it=homing_position.begin();it != homing_position.end();++it) {
                int id = it->first.as<int>();   // <- key
                double pos = it->second.as<double>(); // <- value
                if(_ec_cfg.homing_position.count(id) == 0)
                {
                    if(_ec_cfg.motor_config_map.count(id)>0)
                    {
                        _ec_cfg.homing_position[id]=pos;
                        _ec_cfg.motor_id.push_back(id);
                    }
                    else
                    {
                        throw std::runtime_error("The ID: " + std::to_string(id) + "hasn't a motor configuration, please setup the control mode");
                    }     
                }
                else
                {
                    throw std::runtime_error("The ID: " + std::to_string(id) + " already exists in the homing vector");
                }
            }
        }
        
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
        {
           _ec_cfg.trajectory=ec_cfg["control"]["trajectory"].as<map<int,double>>();
           auto trajectory = ec_cfg["control"]["trajectory"];
           int motor_id_index=0;
           for(YAML::const_iterator it=trajectory.begin();it != trajectory.end();++it) {
                int id = it->first.as<int>(); // <- key
                if(id != _ec_cfg.motor_id[motor_id_index])
                {
                    throw std::runtime_error("The ID: " + std::to_string(id) + " in the trajectory vector doesn't exist or has different position in the homing vector");
                }
                motor_id_index++;
            }
        }
        
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
        
        if(!ec_cfg["control"]["imu_id"])
            _ec_cfg.imu_id.clear();
        else
            _ec_cfg.imu_id=ec_cfg["control"]["imu_id"].as<std::vector<int>>();
    
        if(!ec_cfg["control"]["ft_id"])
            _ec_cfg.ft_id.clear();
        else
            _ec_cfg.ft_id=ec_cfg["control"]["ft_id"].as<std::vector<int>>();
        
        if(!ec_cfg["control"]["pow_id"])
            _ec_cfg.pow_id.clear();
        else
            _ec_cfg.pow_id=ec_cfg["control"]["pow_id"].as<std::vector<int>>();
    }

};

std::map<int,EcUtils::MOTOR_CONFIG> EcUtils::get_motor_config_map(const YAML::Node & motor_cfg,
                                                                  const YAML::Node & joint_map)
{
    std::map<int,EcUtils::MOTOR_CONFIG> motor_config_map;
    auto joint_map_read=joint_map["joint_map"].as<map<int,std::string>>();
    for ( const auto &[esc_id, esc_name] : joint_map_read){
        if(motor_cfg[esc_name]){
            if(motor_config_map.count(esc_id)==0){
                motor_config_map[esc_id].motor_name=esc_name;
                motor_config_map[esc_id].control_mode_type=0x00;
                if(motor_cfg[esc_name]["control_mode"])
                {
                    motor_config_map[esc_id].control_mode_type=motor_cfg[esc_name]["control_mode"].as<int>();
                }

                if(motor_cfg[esc_name]["gains"])
                {
                    motor_config_map[esc_id].gains=motor_cfg[esc_name]["gains"].as<std::vector<float>>();
                }

                if(motor_config_map[esc_id].gains.size()!=5)
                {
                    throw std::runtime_error("Error invalid dimension of impedance gains. It has to be equal to five! ");
                }

                motor_config_map[esc_id].brake_present=false;
                
                if(motor_cfg[esc_name]["brake_present"])
                {
                    motor_config_map[esc_id].brake_present=motor_cfg[esc_name]["brake_present"].as<bool>();
                }
            }
            else{
                throw std::runtime_error("Error: found a duplicated motor configuration on id: "+std::to_string(esc_id));
            }
            
        }
    }
    
    return motor_config_map;
}

EcUtils::EC_CONFIG EcUtils::get_ec_cfg()
{
    return _ec_cfg;
};

std::string EcUtils::get_ec_cfg_file()
{
    return _ec_client_cfg_file;
}

EcIface::Ptr EcUtils::make_ec_iface()
{
    EcIface::Ptr ec_iface_ptr;
    if(_ec_cfg.protocol == "udp")
    {
       auto ec_udp_ptr = std::make_shared<EcUDP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_udp_ptr;
       
    }
    else if(_ec_cfg.protocol == "tcp")
    {
       auto ec_tcp_ptr = std::make_shared<EcTCP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_tcp_ptr;
       
    }
    else if(_ec_cfg.protocol == "iddp")
    {
       auto ec_iddp_ptr = std::make_shared<EcIDDP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_iddp_ptr;
       
    }
    else if(_ec_cfg.protocol == "zipc")
    {
       auto ec_zipc_ptr = std::make_shared<EcZipc>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_zipc_ptr;
       
    }
    else
    {
        throw std::runtime_error("EtherCAT client protocol not recognized, protocols allowed are tcp, udp, ros2, iddp, zipc");
    }
    
    if(ec_iface_ptr != nullptr)
    {
        ec_iface_ptr->start_client(_ec_cfg.period_ms,_ec_cfg.logging); // auto-start 
    }
    
    return ec_iface_ptr;    
}



EcUtils::~EcUtils()
{
    
};

