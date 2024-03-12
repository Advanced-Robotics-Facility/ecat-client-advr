#ifndef EC_COMMON_STEP_H
#define EC_COMMON_STEP_H

#include "utils/ec_utils.h"
#include <utils.h>

class EcCommonStep
{
public:
    
    // Common EtherCAT steps
    EcUtils::EC_CONFIG retrieve_ec_cfg();
    void create_ec(EcIface::Ptr &client,EcUtils::EC_CONFIG &ec_cfg);
    void autodetection(void);
    bool start_ec_motors(std::vector<int> motor_id_vector);
    bool start_ec_motors(void);
    void stop_ec_motors(void);
    bool start_ec_valves(std::vector<int> valve_id_vector);
    bool start_ec_valves(void);
    void stop_ec_valves(void);
    void stop_ec(void);
    
    std::shared_ptr<EcUtils> get_ec_utils();
  
private:
    
    void find_motors(std::vector<int> motor_id_vector);
    void prepare_motors();
    void find_valves(std::vector<int> valve_id_vector);
    
    EcUtils::EC_CONFIG _ec_cfg;
    EcUtils::Ptr _ec_utils;
  
    EcIface::Ptr _client;
    
    MST _motors_start = {};
    PAC _release_brake_cmds = {};
    PAC _engage_brake_cmds = {};
    
    SSI _slave_info;
    std::vector<int> _motor_id_vector;
    std::vector<int> _valve_id_vector;
};

#endif
