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
    if(!ec_cfg_node["network"]["protocol"])
        _ec_cfg.protocol="";
    else
        _ec_cfg.protocol=ec_cfg_node["network"]["protocol"].as<std::string>();

    if(!ec_cfg_node["network"]["hostname"])
        _ec_cfg.host_name="localhost";
    else
        _ec_cfg.host_name=ec_cfg_node["network"]["hostname"].as<std::string>();

    if(!ec_cfg_node["network"]["port"])
        _ec_cfg.host_port=0;
    else
        _ec_cfg.host_port=ec_cfg_node["network"]["port"].as<std::uint32_t>();
    
    if(!ec_cfg_node["period_ms"])
        _ec_cfg.period_ms=100;
    else
        _ec_cfg.period_ms=ec_cfg_node["period_ms"].as<int>();
    
    if(!ec_cfg_node["logging"])
        _ec_cfg.logging=false;
    else
        _ec_cfg.logging=ec_cfg_node["logging"].as<bool>();
    
    if(!ec_cfg_node["auto-start"])
        _ec_cfg.auto_start=true;
    else
        _ec_cfg.auto_start=ec_cfg_node["auto-start"].as<bool>();
    
    
    //****** Trajectory **************//
    if(ec_cfg_node["control"])
    {
        std::string id_map_path="";
        if(ec_cfg_node["control"]["id_map_path"])
        {
            id_map_path=ec_cfg_node["control"]["id_map_path"].as<std::string>();
            try{
                compute_absolute_path(id_map_path);
            }
            catch(std::exception& e){
                throw std::runtime_error(e.what());
            }
        }
        
        std::string motor_config_path="";
        if(ec_cfg_node["control"]["motor_config_path"])
        {
            motor_config_path=ec_cfg_node["control"]["motor_config_path"].as<std::string>();
            try{
                compute_absolute_path(motor_config_path);
            }
            catch(std::exception& e){
                throw std::runtime_error(e.what());
            }
        }
    
        
        if(id_map_path!="")
        {
            auto id_map_node = YAML::LoadFile(id_map_path);
            if(motor_config_path!="")
            {
                std::cout << "Motor configuration: " << motor_config_path << std::endl;
                auto motor_config_node = YAML::LoadFile(motor_config_path);
                _ec_cfg.motor_config_map= get_motor_config_map(motor_config_node,id_map_node);
            }
            else
            {
                throw std::runtime_error("Wrong Motor configuration path!");
            }
        }
        else
        {
            throw std::runtime_error("Wrong Id map path!");
        }
        
        std::vector<int> homing_position_id;
        if(ec_cfg_node["control"]["homing_position"])
        {
           auto homing_position = ec_cfg_node["control"]["homing_position"];
           for(YAML::const_iterator it=homing_position.begin();it != homing_position.end();++it) {
                int id = it->first.as<int>();   // <- key
                double pos = it->second.as<double>(); // <- value
                if(_ec_cfg.homing_position.count(id) == 0)
                {
                    if(_ec_cfg.motor_config_map.count(id)>0)
                    {
                        _ec_cfg.homing_position[id]=pos;
                        homing_position_id.push_back(id);
                    }
                    else
                    {
                        throw std::runtime_error("The ID: " + std::to_string(id) + " hasn't a motor configuration, please setup the control mode");
                    }     
                }
                else
                {
                    throw std::runtime_error("The ID: " + std::to_string(id) + " already exists in the homing vector");
                }
            }
        }

        if(!ec_cfg_node["control"]["homing_time_sec"])
            _ec_cfg.homing_time_sec=60.0; // 60second
        else
            _ec_cfg.homing_time_sec=ec_cfg_node["control"]["homing_time_sec"].as<double>();
        
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.homing_time_sec*1000))
        {
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(ec_cfg_node["control"]["trajectory"])
        {
           _ec_cfg.trajectory=ec_cfg_node["control"]["trajectory"].as<map<int,double>>();
           auto trajectory = ec_cfg_node["control"]["trajectory"];
           int motor_id_index=0;
           for(YAML::const_iterator it=trajectory.begin();it != trajectory.end();++it) {
                int id = it->first.as<int>(); // <- key
                if(id != homing_position_id[motor_id_index])
                {
                    throw std::runtime_error("The ID: " + std::to_string(id) + " in the trajectory vector doesn't exist or has different position in the homing vector");
                }
                motor_id_index++;
            }
            
            if(_ec_cfg.homing_position.size() != _ec_cfg.trajectory.size())
            {
                throw std::runtime_error("Size of homing position vector and trajectory vector is different, please setup the same dimension"); 
            }
        }
        
        if(!ec_cfg_node["control"]["trajectory_time_sec"])
            _ec_cfg.trajectory_time_sec=60.0; // 60 second
        else
            _ec_cfg.trajectory_time_sec=ec_cfg_node["control"]["trajectory_time_sec"].as<double>();
        
        if(2*_ec_cfg.period_ms> (_ec_cfg.trajectory_time_sec*1000))
        {
            throw std::runtime_error("The time of homing time should be at least two times higher then the period_ms variable");
        }
        
        
        if(!ec_cfg_node["control"]["repeat_trj"])
            _ec_cfg.repeat_trj=1;
        else
            _ec_cfg.repeat_trj=ec_cfg_node["control"]["repeat_trj"].as<int>();
        
        if(_ec_cfg.repeat_trj<=0){
            _ec_cfg.repeat_trj=1;
        }
        
        if(!ec_cfg_node["control"]["slave_id_led"])
            _ec_cfg.slave_id_led.clear();
        else
            _ec_cfg.slave_id_led=ec_cfg_node["control"]["slave_id_led"].as<std::vector<int>>();
        
        if(!ec_cfg_node["control"]["imu_id"])
            _ec_cfg.imu_id.clear();
        else
            _ec_cfg.imu_id=ec_cfg_node["control"]["imu_id"].as<std::vector<int>>();
    
        if(!ec_cfg_node["control"]["ft_id"])
            _ec_cfg.ft_id.clear();
        else
            _ec_cfg.ft_id=ec_cfg_node["control"]["ft_id"].as<std::vector<int>>();
        
        if(!ec_cfg_node["control"]["pow_id"])
            _ec_cfg.pow_id.clear();
        else
            _ec_cfg.pow_id=ec_cfg_node["control"]["pow_id"].as<std::vector<int>>();
        
        if(!ec_cfg_node["control"]["valve_id"])
            _ec_cfg.valve_id.clear();
        else
            _ec_cfg.valve_id=ec_cfg_node["control"]["valve_id"].as<std::vector<int>>();
        
        if(!ec_cfg_node["control"]["pump_id"])
            _ec_cfg.pump_id.clear();
        else
            _ec_cfg.pump_id=ec_cfg_node["control"]["pump_id"].as<std::vector<int>>();
        
        generate_fake_slave_info();
    }

};

void EcUtils::generate_fake_slave_info()
{
    int slave_pos=0;
    // MOTOR
    for(int i=0;i<_ec_cfg.motor_id.size();i++){
        int motor_id=_ec_cfg.motor_id[i];
        if(_ec_cfg.motor_config_map.count(motor_id)>0){
            _ec_cfg.fake_slave_info.push_back(std::make_tuple(motor_id,_ec_cfg.motor_config_map[motor_id].type,slave_pos));
            slave_pos++;
        }
    }
    
    // IMU
    slave_pos++;
    for(int i=0;i<_ec_cfg.imu_id.size();i++){
        int imu_id=_ec_cfg.imu_id[i];
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(imu_id,iit::ecat::IMU_ANY,slave_pos));
        slave_pos++;
    }
    
    // FT
    slave_pos++;
    for(int i=0;i<_ec_cfg.ft_id.size();i++){
        int ft_id=_ec_cfg.ft_id[i];
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(ft_id,iit::ecat::FT6_MSP432,slave_pos));
        slave_pos++;
    }
    
    // POW
    slave_pos++;
    for(int i=0;i<_ec_cfg.pow_id.size();i++){
        int pow_id=_ec_cfg.pow_id[i];
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(pow_id,iit::ecat::POW_F28M36_BOARD,slave_pos));
        slave_pos++;
    }
    
    // VALVE
    slave_pos++;
    for(int i=0;i<_ec_cfg.valve_id.size();i++){
        int valve_id=_ec_cfg.valve_id[i];
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(valve_id,iit::ecat::HYQ_KNEE,slave_pos));
        slave_pos++;
    }
    
    // PUMP
    slave_pos++;
    for(int i=0;i<_ec_cfg.pump_id.size();i++){
        int pump_id=_ec_cfg.pump_id[i];
        _ec_cfg.fake_slave_info.push_back(std::make_tuple(pump_id,iit::ecat::HYQ_HPU,slave_pos));
        slave_pos++;
    }
}

void EcUtils::compute_absolute_path(std::string &file_path)
{
    char * dir = strdup(_ec_cfg_file.c_str());
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

std::map<int,EcUtils::MOTOR_CONFIG> EcUtils::get_motor_config_map(const YAML::Node & motor_config_node,
                                                                  const YAML::Node & id_map_node)
{
    std::map<int,EcUtils::MOTOR_CONFIG> motor_config_map;
    auto robot_id_map=id_map_node["joint_map"].as<map<int,std::string>>(); // for xbotinterface
    for ( const auto &[esc_id, esc_name] : robot_id_map){
        if(motor_config_node[esc_name]){
            if(motor_config_map.count(esc_id)==0){
                
                _ec_cfg.motor_id.push_back(esc_id);
                
                motor_config_map[esc_id].motor_name=esc_name;
                
                motor_config_map[esc_id].type=iit::ecat::CENT_AC;
                if(motor_config_node[esc_name]["motor_type"]){
                    std::string type_str=motor_config_node[esc_name]["motor_type"].as<std::string>();
                    
                    if(type_str=="HHCM"){
                        motor_config_map[esc_id].type=iit::ecat::CENT_AC;
                    }
                    else if(type_str=="Circulo"){
                        motor_config_map[esc_id].type=iit::ecat::CIRCULO9;
                    }
                    else if(type_str=="FlexPro"){
                        motor_config_map[esc_id].type=iit::ecat::AMC_FLEXPRO;
                    }
                    else{
                    
                    }
                }
                
                motor_config_map[esc_id].control_mode_type=0x00;
                if(motor_config_node[esc_name]["control_mode"]){
                    motor_config_map[esc_id].control_mode_type=motor_config_node[esc_name]["control_mode"].as<int>();
                }

                if(motor_config_map[esc_id].control_mode_type==0x00){
                    motor_config_map[esc_id].gains={0,0,0,0,0};
                }
                else{
                    if(motor_config_node[esc_name]["gains"]){
                        motor_config_map[esc_id].gains=motor_config_node[esc_name]["gains"].as<std::vector<float>>();
                    }
                }
                
                if(motor_config_map[esc_id].gains.size()!=5){
                        throw std::runtime_error("Error invalid dimension of gains. It has to be equal to five! ");
                }

                motor_config_map[esc_id].brake_present=false;
                
                if(motor_config_node[esc_name]["brake_present"])
                {
                    motor_config_map[esc_id].brake_present=motor_config_node[esc_name]["brake_present"].as<bool>();
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
        if(_ec_cfg.auto_start){
#ifdef TEST_LIBRARY 
            ec_iface_ptr->test_client(_ec_cfg.fake_slave_info);
#endif 
            ec_iface_ptr->start_client(_ec_cfg.period_ms,_ec_cfg.logging); // auto-start
        }
    }
    
    return ec_iface_ptr;    
}



EcUtils::~EcUtils()
{
    
};

