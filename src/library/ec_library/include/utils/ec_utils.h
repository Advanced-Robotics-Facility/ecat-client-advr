#ifndef __EC_UTILS__
#define __EC_UTILS__

#include <yaml-cpp/yaml.h>
#include <thread>
#include "protocol/udp/ec_udp.h"
#include "protocol/tcp/ec_tcp.h"
#include "protocol/ipc/iddp/ec_iddp.h"
#include "protocol/ipc/zipc/ec_zipc.h"

class EcUtils
{
public:
    
    typedef struct DEVICE_CONFIG_t{
        std::string device_name;
        uint32_t type;
        int control_mode_type;
        std::vector<float> gains;
        bool brake_present; // only for motor
    }DEVICE_CONFIG;
        
    typedef struct EC_CONFIG_t{
        
        std::string protocol;
        std::string host_name;
        uint32_t host_port;
        int period_ms; 
        bool logging;
        std::map<int,double> homing_position,trajectory;
        double homing_time_sec,trajectory_time_sec;
        int repeat_trj;
        std::map<int,DEVICE_CONFIG> device_config_map;

        std::vector<int> slave_id_led;
        std::vector<int> motor_id;
        std::vector<int> imu_id;
        std::vector<int> ft_id;
        std::vector<int> pow_id;
        std::vector<int> valve_id;
        std::vector<int> pump_id;
        
        SSI fake_slave_info;
        
    }EC_CONFIG;
    
    typedef std::shared_ptr<EcUtils> Ptr;
    
    EcUtils(EC_CONFIG ec_cfg);
    EcUtils();
    
    ~EcUtils();
    
    EC_CONFIG get_ec_cfg();
    std::string get_ec_cfg_file();
    EcIface::Ptr make_ec_iface(); // EtherCAT Client Interface
private:
    
    std::string _control_mode;
    EC_CONFIG _ec_cfg;
    std::string _ec_cfg_file;
    
    void compute_absolute_path(std::string dir_path,std::string &file_path);
    void generate_fake_slave_info();
    void device_config_map(const YAML::Node & device_config_node,const YAML::Node & robot_id_map_node);

};

#endif
