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
    if(ec_cfg_node["robot"]) {
        int i=0;
        for(const auto &robot_path:_robot_path_v){
            if(ec_cfg_node["robot"][robot_path]){
                _robot_abs_path[i]=ec_cfg_node["robot"][robot_path].as<std::string>();
                try{
                    compute_absolute_path(_ec_cfg_file,_robot_abs_path[i]);
                    if(_robot_abs_path[i]==""){
                        throw std::runtime_error("Wrong "+robot_path+" path!");
                    }
                    i++;
                }catch(std::exception& e){
                    throw std::runtime_error(e.what());
                }
            }
        }
       
        if(_robot_abs_path[0]!="" && _robot_abs_path[1]!=""){
            try{
                std::cout << "Robot id map: " << _robot_abs_path[0] << std::endl;
                _robot_id_map_node = YAML::LoadFile(_robot_abs_path[0]);
                std::cout << "Robot control: " << _robot_abs_path[1] << std::endl;
                _robot_control_node = YAML::LoadFile(_robot_abs_path[1]);

                config_device();
                config_trajectory();
                trajectory_generator();
                generate_fake_slave_info();

            }catch(std::exception& e){
                throw std::runtime_error(e.what());
            }
        }
    }
};

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

void EcUtils::device_config_map(const YAML::Node & device_config_node)
{
    auto robot_id_map=_robot_id_map_node["joint_map"].as<map<int,std::string>>(); // for xbotinterface
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

void EcUtils::config_device()
{
    _ec_cfg.device_config_map.clear();

    for(const auto &device_type:_device_type_vector){
        if(_robot_control_node[device_type]){
            if(_robot_control_node[device_type]["config_path"]){
                try{
                    std::string config_path=_robot_control_node[device_type]["config_path"].as<std::string>();
                    std::string config_path_abs=config_path;
                    compute_absolute_path(_robot_abs_path[1],config_path_abs);
                    if(config_path_abs!=""){
                        std::cout <<  device_type +" configuration path: " << config_path_abs<< std::endl;
                        auto config_node = YAML::LoadFile(config_path_abs);
                        device_config_map(config_node);
                    }else{
                        throw std::runtime_error("Wrong "+config_path+" path!");
                    }
                }catch(std::exception& e){
                    throw std::runtime_error(e.what());
                }
            }
        }
    }
}

void EcUtils::config_trajectory()
{
    if(_robot_control_node["trajectory"]["type"]){
        _ec_cfg.trj_type=_robot_control_node["trajectory"]["type"].as<std::string>();
    }
    if(_robot_control_node["trajectory"]["frequency"]){
        _ec_cfg.trj_time=1/(_robot_control_node["trajectory"]["frequency"].as<float>());
        if(2*_ec_cfg.period_ms> (_ec_cfg.trj_time*1000)){
            throw std::runtime_error("The trajectoy time should be at least two times higher then the period_ms variable");
        }
    }
    if(_robot_control_node["trajectory"]["repeat"]){
        _ec_cfg.repeat_trj=_robot_control_node["trajectory"]["repeat"].as<int>();
    }
    
    for(const auto &device_type:_device_type_vector){
        if(_robot_control_node[device_type]){
            if(_robot_control_node[device_type]["id"]){
                auto id_vector = _robot_control_node[device_type]["id"].as<std::vector<int>>();

                for(const auto &id: id_vector){
                    if(_ec_cfg.device_config_map.count(id)==0 && device_type!="pump"){
                        throw std::runtime_error("The ID: " + std::to_string(id) + " hasn't a " +  device_type + " configuration, please setup the control mode");
                    }
                }

                std::map<std::string,double> set_point;
                if(_robot_control_node[device_type]["set_point"]){
                    set_point = _robot_control_node[device_type]["set_point"].as<std::map<std::string,double>>();
                }
                else{
                    throw std::runtime_error("Fatal error: cannot find the set point for the device");
                }

                std::vector<double> homing,trajectory;
                if(_robot_control_node[device_type]["homing"]){
                    homing = _robot_control_node[device_type]["homing"].as<std::vector<double>>();
                }

                if(_robot_control_node[device_type]["trajectory"]){
                    trajectory = _robot_control_node[device_type]["trajectory"].as<std::vector<double>>();
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
}

void EcUtils::trajectory_generator()
{
    double trj_time = _ec_cfg.trj_time * _ec_cfg.repeat_trj;
    for(const auto &device_type:_device_type_vector){
        for(const auto &[id,set_poit_map]:_ec_cfg.trj_config_map[device_type].set_point){
            for(const auto &[ctrl_type,set_poit]:set_poit_map){
                if ( _ec_cfg.trj_type=="polynomial"){
                    return;
                }
                else{
                    // Spline trajectory
                    if ( _ec_cfg.trj_type=="spline"){
                        std::vector<double> time =  {0,trj_time};
                        std::vector<double> value = {0,set_poit};
                        //_ec_cfg.trj_config_map[device_type].trj_generator[id][ctrl_type] = std::make_shared<Spline_trajectory>(time, value);
                    } 
                    // Smoother trajectory
                    else if ( _ec_cfg.trj_type=="smoother"){
                        std::vector<double> time =  {0,trj_time};
                        std::vector<double> value = {0,set_poit};
                        _ec_cfg.trj_config_map[device_type].trj_generator[id][ctrl_type] = std::make_shared<Smoother_trajectory>(time, value);
                    } 
                    // Smooth step
                    else if ( _ec_cfg.trj_type=="step"){
                        std::vector<double> time =  {0,trj_time};
                        double amplitude = set_poit;
                        double freq = 1/_ec_cfg.trj_time;
                        double theta = 0.0;
                        _ec_cfg.trj_config_map[device_type].trj_generator[id][ctrl_type] = std::make_shared<Steps_trajectory>(freq, amplitude,theta,time);
                    } 
                    // Sine trajectory
                    else if ( _ec_cfg.trj_type=="sine"){
                        std::vector<double> time =  {0,trj_time};
                        double amplitude = set_poit;
                        double freq = 1/_ec_cfg.trj_time;
                        double theta = 0.0;
                        _ec_cfg.trj_config_map[device_type].trj_generator[id][ctrl_type] = std::make_shared<Sine_trajectory>(freq, amplitude, theta, time);
                    } 
                    // Stair trajectory
                    else if ( _ec_cfg.trj_type=="stair"){

                    } 
                    // Chirp trajectory
                    else if ( _ec_cfg.trj_type=="chirp"){

                    }
                    else{
                        throw std::runtime_error("Trajectory type not recognized!");
                    }
                }
            }
        }
    }
}

void EcUtils::generate_fake_slave_info()
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

    if(_robot_control_node["simulation"]) {
        if(_robot_control_node["simulation"]["imu_id"]){
            // IMU
            auto imu_id_v=_robot_control_node["simulation"]["imu_id"].as<std::vector<int>>();
            for(const auto &imu_id:imu_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(imu_id,iit::ecat::IMU_ANY,slave_pos));
                slave_pos++;
            }
        }

        if(_robot_control_node["simulation"]["ft_id"]){
            // FT
            auto ft_id_v=_robot_control_node["simulation"]["ft_id"].as<std::vector<int>>();
            for(const auto &ft_id:ft_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(ft_id,iit::ecat::FT6_MSP432,slave_pos));
                slave_pos++;
            }
        }
        
        if(_robot_control_node["simulation"]["pow_id"]){
            // POW
            auto pow_id_v=_robot_control_node["simulation"]["pow_id"].as<std::vector<int>>();
            for(const auto &pow_id:pow_id_v){
                _ec_cfg.fake_slave_info.push_back(std::make_tuple(pow_id,iit::ecat::POW_F28M36_BOARD,slave_pos));
                slave_pos++;
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

