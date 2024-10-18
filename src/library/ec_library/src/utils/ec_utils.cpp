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
    
    if (_ec_cfg_file.c_str()==NULL){
        throw std::runtime_error("EtherCAT Client configuration not found, setup the environment variable: EC_CFG ");
    }
    
    auto ec_cfg_node=YAML::LoadFile(_ec_cfg_file);

    // Read the EC Client configuration.
    
    //****** Network **************//
    if(ec_cfg_node["network"]){
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
    }
    else{
        throw std::runtime_error("Missing network setup!");
    }
    
    //****** General **************//
    if(!ec_cfg_node["general"]["period_ms"]){
        _ec_cfg.period_ms=100;
    }else{
        _ec_cfg.period_ms=ec_cfg_node["general"]["period_ms"].as<int>();
    }
    
    if(!ec_cfg_node["general"]["logging"]){
            _ec_cfg.logging=false;
    }else{
        _ec_cfg.logging=ec_cfg_node["general"]["logging"].as<bool>();
    }

    //****** Control **************//
    if(ec_cfg_node["control"]) {
        std::vector<std::string> ctrl_path_vector={"robot_id_map_path","robot_config_path","robot_trajectory_path"};
        std::vector<std::string> ctrl_abs_path={"","",""};

        int i=0;
        for(const auto &ctrl_path:ctrl_path_vector){
            if(ec_cfg_node["control"][ctrl_path]){
                ctrl_abs_path[i]=ec_cfg_node["control"][ctrl_path].as<std::string>();
                try{
                    compute_absolute_path(_ec_cfg_file,ctrl_abs_path[i]);
                    if(ctrl_abs_path[i]==""){
                        throw std::runtime_error("Wrong "+ctrl_path+"!");
                    }
                    i++;
                }catch(std::exception& e){
                    throw std::runtime_error(e.what());
                }
            }
        }
       
        if(ctrl_abs_path[0]!="" && ctrl_abs_path[1]!=""){

            auto robot_id_map_node = YAML::LoadFile(ctrl_abs_path[0]);
            std::cout << "Robot configuration: " << ctrl_abs_path[1] << std::endl;
            auto robot_config_node = YAML::LoadFile(ctrl_abs_path[1]);

            _ec_cfg.device_config_map.clear();
            std::vector<std::string> config_path_vector={"motor_config_path","valve_config_path"};
            std::vector<std::string> config_abs_path={"",""};

            int i=0;
            for(const auto &config_path:config_path_vector){
                if(robot_config_node["robot_cfg"][config_path]){
                    config_abs_path[i]=robot_config_node["robot_cfg"][config_path].as<std::string>();
                    compute_absolute_path(ctrl_abs_path[1],config_abs_path[i]);
                    if(config_abs_path[i]!=""){
                        std::cout <<  config_path+": " << config_abs_path[i] << std::endl;
                        auto config_node = YAML::LoadFile(config_abs_path[i]);
                        device_config_map(config_node,robot_id_map_node);
                        i++;
                    }else{
                        throw std::runtime_error("Wrong "+config_path+"!");
                    }
                }
            }

            if(ctrl_abs_path[2]!=""){
                std::cout << "Robot trajectory: " << ctrl_abs_path[2] << std::endl;
                auto robot_trajectory_node = YAML::LoadFile(ctrl_abs_path[2]);
                try{
                    config_trj(robot_trajectory_node);
                }catch(std::exception& e){
                    throw std::runtime_error(e.what());
                }
            }
        }
    }
    
    generate_fake_slave_info(ec_cfg_node);

};

void EcUtils::generate_fake_slave_info(const YAML::Node & ec_cfg_node)
{
    int slave_pos=1;
    
    // Motor
    if(_ec_cfg.trj_config_map.count("motor")>0){
        for(const auto &[id,set_point]:_ec_cfg.trj_config_map["motor"].set_point){
            _ec_cfg.fake_slave_info.push_back(std::make_tuple(id,_ec_cfg.device_config_map[id].type,slave_pos));
            slave_pos++;
        }
    }

    // Valve
    if(_ec_cfg.trj_config_map.count("valve")>0){
        for(const auto &[id,set_point]:_ec_cfg.trj_config_map["valve"].set_point){
            _ec_cfg.fake_slave_info.push_back(std::make_tuple(id,iit::ecat::HYQ_KNEE,slave_pos));
            slave_pos++;
        }
    }

    // Pump
    if(_ec_cfg.trj_config_map.count("pump")>0){
        for(const auto &[id,set_point]:_ec_cfg.trj_config_map["pump"].set_point){
            _ec_cfg.fake_slave_info.push_back(std::make_tuple(id,iit::ecat::HYQ_HPU,slave_pos));
            slave_pos++;
        }
    }

    if(ec_cfg_node["simulation"]) {
        if(ec_cfg_node["simulation"]["imu_id"]){
            // IMU
            auto imu_id_v=ec_cfg_node["simulation"]["imu_id"].as<std::vector<int>>();
            for(const auto &imu_id:imu_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(imu_id,iit::ecat::IMU_ANY,slave_pos));
                slave_pos++;
            }
        }

        if(ec_cfg_node["simulation"]["ft_id"]){
            // FT
            auto ft_id_v=ec_cfg_node["simulation"]["ft_id"].as<std::vector<int>>();
            for(const auto &ft_id:ft_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(ft_id,iit::ecat::FT6_MSP432,slave_pos));
                slave_pos++;
            }
        }
        
        if(ec_cfg_node["simulation"]["pow_id"]){
            // POW
            auto pow_id_v=ec_cfg_node["simulation"]["pow_id"].as<std::vector<int>>();
            for(const auto &pow_id:pow_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(pow_id,iit::ecat::POW_F28M36_BOARD,slave_pos));
                slave_pos++;
            }
        }
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

    if(retcode != 0){
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
                }else{
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
                    }else if(type_str=="Synapticon"){
                        _ec_cfg.device_config_map[esc_id].type=iit::ecat::SYNAPTICON_v5_1;
                    }else{
                        _ec_cfg.device_config_map[esc_id].type=iit::ecat::CENT_AC;
                    }
                }

                _ec_cfg.device_config_map[esc_id].brake_present=false;
                if(device_config_node[esc_name]["brake_present"]){
                    _ec_cfg.device_config_map[esc_id].brake_present=device_config_node[esc_name]["brake_present"].as<bool>();
                }
            }else{
                throw std::runtime_error("Error: found a duplicated device configuration on id: "+std::to_string(esc_id));
            }
        }
    }
}

void EcUtils::config_trj(const YAML::Node & robot_trajectory_node)
{
    if(robot_trajectory_node["trajectory"]["type"]){
        _ec_cfg.trj_type=robot_trajectory_node["trajectory"]["type"].as<std::string>();
    }
    if(robot_trajectory_node["trajectory"]["frequency"]){
        _ec_cfg.trj_time=1/(robot_trajectory_node["trajectory"]["frequency"].as<float>());
        if(2*_ec_cfg.period_ms> (_ec_cfg.trj_time*1000)){
            throw std::runtime_error("The trajectoy time should be at least two times higher then the period_ms variable");
        }
    }
    if(robot_trajectory_node["trajectory"]["repeat"]){
        _ec_cfg.repeat_trj=robot_trajectory_node["trajectory"]["repeat"].as<int>();
    }

    std::vector<std::string> device_type_vector={"motor","valve","pump"};          
    for(const auto &device_type:device_type_vector){
        if(robot_trajectory_node[device_type]["id"]){
            auto id_vector = robot_trajectory_node[device_type]["id"].as<std::vector<int>>();

            for(const auto &id: id_vector){
                if(_ec_cfg.device_config_map.count(id)==0 && device_type!="pump"){
                    throw std::runtime_error("The ID: " + std::to_string(id) + " hasn't a " +  device_type + " configuration, please setup the control mode");
                }
            }

            std::map<std::string,double> set_point;
            if(robot_trajectory_node[device_type]["set_point"]){
                set_point = robot_trajectory_node[device_type]["set_point"].as<std::map<std::string,double>>();
            }
            else{
                throw std::runtime_error("Fatal error: cannot find the set point for the device");
            }

            std::vector<double> homing,trajectory;
            if(robot_trajectory_node[device_type]["homing"]){
                homing = robot_trajectory_node[device_type]["homing"].as<std::vector<double>>();
            }

            if(robot_trajectory_node[device_type]["trajectory"]){
                trajectory = robot_trajectory_node[device_type]["trajectory"].as<std::vector<double>>();
            }

            if(id_vector.size() != homing.size() && !homing.empty()){
                throw std::runtime_error("Motor id size has different size of homing vector");
            }
            else{
                if(homing.size() != trajectory.size() ){
                    throw std::runtime_error("Homing vector size has different size of trajectory vector");
                }
            }

            int i=0;
            for(const auto &id:id_vector){
                if(_ec_cfg.trj_config_map[device_type].set_point.count(id)==0){
                    _ec_cfg.trj_config_map[device_type].set_point[id]=set_point;
                    if(!homing.empty() && !trajectory.empty()){
                        _ec_cfg.trj_config_map[device_type].homing[id]=homing[i];
                        _ec_cfg.trj_config_map[device_type].trajectory[id]=trajectory[i];
                        i++;
                    }
                }else{
                    throw std::runtime_error(device_type + " id duplicated!");     
                }
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
    if(_ec_cfg.protocol == "udp"){
       auto ec_udp_ptr = std::make_shared<EcUDP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_udp_ptr;
       
    }else if(_ec_cfg.protocol == "tcp"){
       auto ec_tcp_ptr = std::make_shared<EcTCP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_tcp_ptr;
       
    }else if(_ec_cfg.protocol == "iddp"){
       auto ec_iddp_ptr = std::make_shared<EcIDDP>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_iddp_ptr;
       
    }else if(_ec_cfg.protocol == "zipc"){
       auto ec_zipc_ptr = std::make_shared<EcZipc>(_ec_cfg.host_name,_ec_cfg.host_port);
       ec_iface_ptr = ec_zipc_ptr;
       
    }else{
        throw std::runtime_error("EtherCAT client protocol not recognized, protocols allowed are tcp, udp, ros2, iddp, zipc");
    }
    
    if(ec_iface_ptr != nullptr){
#ifdef TEST_LIBRARY 
        ec_iface_ptr->set_slaves_info(_ec_cfg.fake_slave_info);
#endif 
    }
    
    return ec_iface_ptr;    
}



EcUtils::~EcUtils()
{
    
};

