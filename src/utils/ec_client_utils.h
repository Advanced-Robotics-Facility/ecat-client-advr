#ifndef __EC_CLIENT_UTILS__
#define __EC_CLIENT_UTILS__

#include <yaml-cpp/yaml.h>


class EC_Client_Utils
{
public:
    
    typedef std::shared_ptr<EC_Client_Utils> Ptr;
    
    EC_Client_Utils(const YAML::Node & ec_client_cfg);
    
    ~EC_Client_Utils();
    
    typedef struct EC_CONFIG_t{
        
        int UDP_period_ms; 
        std::map<double,double> homing_position,trajectory;
        int homing_time_sec,trajectory_time_sec;
        int repeat_trj;
        std::string host_name_s;
        uint32_t host_port;
        int control_mode_type;
        std::vector<float> gains;
        std::vector<int> slave_id_led;
        
    }EC_CONFIG;
    
    EC_CONFIG get_ec_client_config();

private:
    
    std::string _control_mode;
    std::vector<int> _success_cmd_slave,_unsuccess_cmd_slave;
    EC_CONFIG _ec_config;
  
};

#endif
