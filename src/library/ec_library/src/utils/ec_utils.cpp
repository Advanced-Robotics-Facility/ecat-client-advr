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
    _ec_cfg_file= getenv ("EC_CFG");
    
    if (_ec_cfg_file.c_str()==NULL)
    {
        throw std::runtime_error("EtherCAT Client configuration not found, setup the environment variable: EC_CFG ");
    }
    
    auto ec_cfg_node=YAML::LoadFile(_ec_cfg_file);
    
    // Read the EC Client configuration.
    
    //****** Protocol **************//
    if(!ec_cfg_node["network"]["protocol"]){
        _ec_cfg.protocol="";
    }else{
        _ec_cfg.protocol=ec_cfg_node["network"]["protocol"].as<std::string>();
    }

    if(!ec_cfg_node["network"]["hostname"]){
        _ec_cfg.host_name="localhost";
    } else{
        _ec_cfg.host_name=ec_cfg_node["network"]["hostname"].as<std::string>();
    }

    if(!ec_cfg_node["network"]["port"]){
        _ec_cfg.host_port=0;
    }else{
        _ec_cfg.host_port=ec_cfg_node["network"]["port"].as<std::uint32_t>();
    }
    
    if(!ec_cfg_node["period_ms"]){
        _ec_cfg.period_ms=100;
    }else{
        _ec_cfg.period_ms=ec_cfg_node["period_ms"].as<int>();
    }
    
    if(!ec_cfg_node["logging"]){
            _ec_cfg.logging=false;
    }else{
        _ec_cfg.logging=ec_cfg_node["logging"].as<bool>();
    }
    
    
    //****** Control **************//
    if(ec_cfg_node["control"])
    {
        std::string robot_id_map_path="";
        if(ec_cfg_node["control"]["robot_id_map_path"]){
            robot_id_map_path=ec_cfg_node["control"]["robot_id_map_path"].as<std::string>();
            try{
                compute_absolute_path(_ec_cfg_file,robot_id_map_path);
                if(robot_id_map_path==""){
                    throw std::runtime_error("Wrong robot id map path!");
                }
            }catch(std::exception& e){
                throw std::runtime_error(e.what());
            }
        }
        
        std::string robot_config_path="";
        if(ec_cfg_node["control"]["robot_config_path"]){
            robot_config_path=ec_cfg_node["control"]["robot_config_path"].as<std::string>();
            try{
                compute_absolute_path(_ec_cfg_file,robot_config_path);
                if(robot_config_path==""){
                    throw std::runtime_error("Wrong robot configuration path!");
                }
            }catch(std::exception& e){
                throw std::runtime_error(e.what());
            }
        }
    
        
        if(robot_id_map_path!="" && robot_config_path!=""){

            auto robot_id_map_node = YAML::LoadFile(robot_id_map_path);
            std::cout << "Robot configuration: " << robot_config_path << std::endl;
            auto robot_config_node = YAML::LoadFile(robot_config_path);
            std::string motor_config_path="",valve_config_path="";
            _ec_cfg.device_config_map.clear();

            if(robot_config_node["robot_cfg"]["motor_config_path"]){
                motor_config_path=robot_config_node["robot_cfg"]["motor_config_path"].as<std::string>();
                compute_absolute_path(robot_config_path,motor_config_path);
                if(motor_config_path!=""){
                    std::cout << "Motor configuration: " << motor_config_path << std::endl;
                    auto motor_config_node = YAML::LoadFile(motor_config_path);
                    device_config_map(motor_config_node,robot_id_map_node);
                }else{
                    throw std::runtime_error("Wrong motor configuration path!");
                }
            }

            if(robot_config_node["robot_cfg"]["valve_config_path"]){
                valve_config_path=robot_config_node["robot_cfg"]["valve_config_path"].as<std::string>();
                compute_absolute_path(robot_config_path,valve_config_path);
                if(valve_config_path!=""){
                    std::cout << "Valve configuration: " << valve_config_path << std::endl;
                    auto valve_config_node = YAML::LoadFile(valve_config_path);
                    device_config_map(valve_config_node,robot_id_map_node);
                }else{
                    throw std::runtime_error("Wrong valve configuration path!");
                }
            }
        }
        if(ec_cfg_node["control"]["homing_position"]){
           auto homing_position = ec_cfg_node["control"]["homing_position"];
           for(YAML::const_iterator it=homing_position.begin();it != homing_position.end();++it) {
                int id = it->first.as<int>();   // <- key
                double pos = it->second.as<double>(); // <- value
                if(_ec_cfg.homing_position.count(id) == 0){
                    if(_ec_cfg.device_config_map.count(id)>0){
                        _ec_cfg.homing_position[id]=pos;
                        _ec_cfg.motor_id.push_back(id);
                    }else{
                        throw std::runtime_error("The ID: " + std::to_string(id) + " hasn't a motor configuration, please setup the control mode");
                    }     
                }else{
                    throw std::runtime_error("The ID: " + std::to_string(id) + " already exists in the homing vector");
                }
            }
        }

        if(!ec_cfg_node["control"]["homing_time_sec"]){
            _ec_cfg.homing_time_sec=60.0; // 60second
        }else{
            _ec_cfg.homing_time_sec=ec_cfg_node["control"]["homing_time_sec"].as<double>();
        }
        
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.homing_time_sec*1000)){
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(ec_cfg_node["control"]["trajectory"]){
           _ec_cfg.trajectory=ec_cfg_node["control"]["trajectory"].as<map<int,double>>();
           auto trajectory = ec_cfg_node["control"]["trajectory"];
           int motor_id_index=0;
           for(YAML::const_iterator it=trajectory.begin();it != trajectory.end();++it) {
                int id = it->first.as<int>(); // <- key
                if(id != _ec_cfg.motor_id[motor_id_index]){
                    throw std::runtime_error("The ID: " + std::to_string(id) + " in the trajectory vector doesn't exist or has different position in the homing vector");
                }
                motor_id_index++;
            }
            
            if(_ec_cfg.homing_position.size() != _ec_cfg.trajectory.size()){
                throw std::runtime_error("Size of homing position vector and trajectory vector is different, please setup the same dimension"); 
            }
        }
        
        if(!ec_cfg_node["control"]["trajectory_time_sec"]){
            _ec_cfg.trajectory_time_sec=60.0; // 60 second
        }else{
            _ec_cfg.trajectory_time_sec=ec_cfg_node["control"]["trajectory_time_sec"].as<double>();
        }
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.trajectory_time_sec*1000)){
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(!ec_cfg_node["control"]["repeat_trj"]){
            _ec_cfg.repeat_trj=1;
        }else{
            _ec_cfg.repeat_trj=ec_cfg_node["control"]["repeat_trj"].as<int>();
        }
        
        if(_ec_cfg.repeat_trj<=0){
            _ec_cfg.repeat_trj=1;
        }
        
        if(!ec_cfg_node["control"]["slave_id_led"]){
            _ec_cfg.slave_id_led.clear();
        }else{
            _ec_cfg.slave_id_led=ec_cfg_node["control"]["slave_id_led"].as<std::vector<int>>();
        }
        
        if(!ec_cfg_node["control"]["imu_id"]){
            _ec_cfg.imu_id.clear();
        }else{
            _ec_cfg.imu_id=ec_cfg_node["control"]["imu_id"].as<std::vector<int>>();
        }
    
        if(!ec_cfg_node["control"]["ft_id"]){
            _ec_cfg.ft_id.clear();
        }else{
            _ec_cfg.ft_id=ec_cfg_node["control"]["ft_id"].as<std::vector<int>>();
        }
        
        if(!ec_cfg_node["control"]["pow_id"]){
            _ec_cfg.pow_id.clear();
        }else{
            _ec_cfg.pow_id=ec_cfg_node["control"]["pow_id"].as<std::vector<int>>();
        }
    
        if(!ec_cfg_node["control"]["valve_id"]){
            _ec_cfg.valve_id.clear();
        }else{
            _ec_cfg.valve_id=ec_cfg_node["control"]["valve_id"].as<std::vector<int>>();
            for (const auto &valve_id:_ec_cfg.valve_id){
                if(_ec_cfg.device_config_map.count(valve_id)==0){
                    throw std::runtime_error("The ID: " + std::to_string(valve_id) + " hasn't a valve configuration, please setup the control mode");
                }
            }
        }
        
        if(!ec_cfg_node["control"]["pump_id"]){
            _ec_cfg.pump_id.clear();
        }else{
            _ec_cfg.pump_id=ec_cfg_node["control"]["pump_id"].as<std::vector<int>>();
        }
        
        generate_fake_slave_info();
    }

};

void EcUtils::generate_fake_slave_info()
{
    int slave_pos=1;
    // MOTOR
    for(const auto &motor_id:_ec_cfg.motor_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(motor_id,_ec_cfg.device_config_map[motor_id].type,slave_pos));
        slave_pos++;
    }
    
    // IMU
    for(const auto &imu_id:_ec_cfg.imu_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(imu_id,iit::ecat::IMU_ANY,slave_pos));
        slave_pos++;
    }
    
    // FT
    for(const auto &ft_id:_ec_cfg.ft_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(ft_id,iit::ecat::FT6_MSP432,slave_pos));
        slave_pos++;
    }
    
    // POW
    for(const auto &pow_id:_ec_cfg.pow_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(pow_id,iit::ecat::POW_F28M36_BOARD,slave_pos));
        slave_pos++;
    }
    
    // VALVE
    for(const auto &valve_id:_ec_cfg.valve_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(valve_id,iit::ecat::HYQ_KNEE,slave_pos));
        slave_pos++;
    }
    
    // PUMP
    for(const auto &pump_id:_ec_cfg.pump_id){
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(pump_id,iit::ecat::HYQ_HPU,slave_pos));
        slave_pos++;
    }
}

void EcUtils::compute_absolute_path(std::string dir_path,std::string &file_path)
{
    char * dir = strdup(dir_path.c_str());
    std::string dirn_name= dirname(dir);
    std::string cmd = "set -eu; cd " + dirn_name + "; /bin/echo " + file_path;

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe){
        throw std::runtime_error("popen() failed");
    }

    std::string result;

    try{
        char buffer[128];

        while(fgets(buffer, sizeof buffer, pipe) != NULL){
            result += buffer;
        }
    }catch(...){
        pclose(pipe);
        throw;
    }

    int retcode = pclose(pipe);

    if(retcode != 0)
    {
        throw std::runtime_error("child process '" + std::string(cmd) + "' exited with code " + std::to_string(retcode));
    }
    
    result = result.substr(0, result.size() - 1);
    
    file_path=result;
}

void EcUtils::device_config_map(const YAML::Node & device_config_node,const YAML::Node & robot_id_map_node)
{
    auto robot_id_map=robot_id_map_node["joint_map"].as<map<int,std::string>>(); // for xbotinterface
    for ( const auto &[esc_id, esc_name] : robot_id_map){
        if(device_config_node[esc_name]){
            if(_ec_cfg.device_config_map.count(esc_id)==0){

                _ec_cfg.device_config_map[esc_id].device_name=esc_name;

                _ec_cfg.device_config_map[esc_id].control_mode_type=0x00;
                if(device_config_node[esc_name]["control_mode"]){
                    _ec_cfg.device_config_map[esc_id].control_mode_type=device_config_node[esc_name]["control_mode"].as<int>();
                }

                if(_ec_cfg.device_config_map[esc_id].control_mode_type==0x00){
                    _ec_cfg.device_config_map[esc_id].gains={0,0,0,0,0};
                }
                else{
                    if(device_config_node[esc_name]["gains"]){
                        _ec_cfg.device_config_map[esc_id].gains=device_config_node[esc_name]["gains"].as<std::vector<float>>();
                    }
                }
                
                if(_ec_cfg.device_config_map[esc_id].gains.size()!=5){
                        throw std::runtime_error("Error invalid dimension of gains. It has to be equal to five! ");
                }
                
                if(device_config_node[esc_name]["motor_type"]){
                    std::string type_str=device_config_node[esc_name]["motor_type"].as<std::string>();
                    
                    if(type_str=="ADVRF"){
                        _ec_cfg.device_config_map[esc_id].type=iit::ecat::CENT_AC;
                    }
                    else if(type_str=="Synapticon"){
                        _ec_cfg.device_config_map[esc_id].type=iit::ecat::SYNAPTICON_v5_1;
                    }
                    else{
                        _ec_cfg.device_config_map[esc_id].type=iit::ecat::CENT_AC;
                    }
                }

                _ec_cfg.device_config_map[esc_id].brake_present=false;
                if(device_config_node[esc_name]["brake_present"]){
                    _ec_cfg.device_config_map[esc_id].brake_present=device_config_node[esc_name]["brake_present"].as<bool>();
                }
            }
            else{
                throw std::runtime_error("Error: found a duplicated device configuration on id: "+std::to_string(esc_id));
            }
            
        }
    }
}

EcUtils::EC_CONFIG EcUtils::get_ec_cfg()
{
    return _ec_cfg;
};

std::string EcUtils::get_ec_cfg_file()
{
    return _ec_cfg_file;
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
#ifdef TEST_LIBRARY 
        ec_iface_ptr->set_slaves_info(_ec_cfg.fake_slave_info);
#endif 
    }
    
    return ec_iface_ptr;    
}



EcUtils::~EcUtils()
{
    
};

