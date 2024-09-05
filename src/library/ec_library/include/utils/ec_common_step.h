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

    void set_motors_id(std::vector<int> motor_start_vector);
    void set_valves_id(std::vector<int> valve_start_vector);
    
    bool start_ec_sys(void);
    void stop_ec_sys(void);
    void telemetry();
    
    std::shared_ptr<EcUtils> get_ec_utils();
  
private:
    
    void find_motors();
    void prepare_motors();
    void find_valves();
    
    void autodetection(void);
    bool start_ec_motors(void);
    void stop_ec_motors(void);
    bool start_ec_valves(void);
    void stop_ec_valves(void);
    
    EcUtils::EC_CONFIG _ec_cfg;
    EcUtils::Ptr _ec_utils;
  
    EcIface::Ptr _client;
    
    MST _motors_start = {};
    PAC _release_brake_cmds = {};
    PAC _engage_brake_cmds = {};
    
    SSI _slave_info;
    std::vector<int> _motor_id_vector,_motor_start_vector;
    std::vector<int> _valve_id_vector,_valve_start_vector;
    bool _print_telemetry=false;
    bool _start_motor=false,_start_valve=false;

    //Power board
    PwrStatusMap _pow_status_map;
    //IMU
    ImuStatusMap _imu_status_map;
    // Pump
    PumpStatusMap _pump_status_map;
    // Valve
    ValveStatusMap _valve_status_map;
    // Motor
    MotorStatusMap _motors_status_map;
};

#endif
