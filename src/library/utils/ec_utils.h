#ifndef __EC_UTILS__
#define __EC_UTILS__

#include <yaml-cpp/yaml.h>
#include <thread>
#include "ec_udp.h"

class EcUtils
{
public:
    
    typedef std::shared_ptr<EcUtils> Ptr;
    
    EcUtils(const YAML::Node & ec_cfg);
    
    ~EcUtils();
    
    typedef struct EC_CONFIG_t{
        
        std::string protocol;
        std::string host_name;
        uint32_t host_port;
        int period_ms; 
        std::map<double,double> homing_position,trajectory;
        int homing_time_sec,trajectory_time_sec;
        int repeat_trj;
        int control_mode_type;
        std::vector<float> gains;
        std::vector<int> slave_id_led;
        
    }EC_CONFIG;
    
    EC_CONFIG get_ec_cfg();
    EcWrapper::Ptr make_ecat_client();
private:
    
    std::string _control_mode;
    EC_CONFIG _ec_cfg;
    std::thread *_ec_thread;
  
};

#endif
