#ifndef __EC_UTILS__
#define __EC_UTILS__

#include <yaml-cpp/yaml.h>
#include <thread>
#include "ec_udp.h"
#include "ec_tcp.h"
#include "ec_iddp.h"

class EcUtils
{
public:
    
    typedef struct EC_CONFIG_t{
        
        std::string protocol;
        std::string host_name;
        uint32_t host_port;
        int period_ms; 
        bool logging;
        std::map<int,double> homing_position,trajectory;
        int homing_time_sec,trajectory_time_sec;
        int repeat_trj;
        int control_mode_type;
        std::vector<float> gains;
        std::vector<int> slave_id_led;
        std::vector<int> motor_id;
        std::vector<int> imu_id;
        std::vector<int> ft_id;
        std::vector<int> pow_id;
        
    }EC_CONFIG;
    
    typedef std::shared_ptr<EcUtils> Ptr;
    
    EcUtils(EC_CONFIG ec_cfg);
    EcUtils(const YAML::Node & ec_cfg);
    
    ~EcUtils();
    
    EC_CONFIG get_ec_cfg();
    EcIface::Ptr make_ec_iface(); // EtherCAT Client Interface
private:
    
    std::string _control_mode;
    EC_CONFIG _ec_cfg;
};

#endif
