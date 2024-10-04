#ifndef EC_WRAPPER_H
#define EC_WRAPPER_H

#include "utils/ec_utils.h"
#include <utils.h>

//Power board
extern PwrStatusMap pow_status_map;
//IMU
extern ImuStatusMap imu_status_map;
// Pump
extern PumpStatusMap pump_status_map;
extern PumpReferenceMap pumps_ref;
// Valve
extern ValveStatusMap valve_status_map;
extern ValveReferenceMap valves_ref;
// Motor
extern MotorStatusMap motors_status_map;
extern MotorReferenceMap motors_ref;

class EcWrapper
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
    void ec_self_sched(std::string thread_name="Thread"); // Scheduling an external thread/process with same policy, cpu and less priority of the ec client.
  
private:
    
    void find_motors();
    void prepare_motors();
    void find_valves();
    
    void autodetection(void);
    bool start_ec_motors(void);
    void stop_ec_motors(void);
    bool start_ec_valves(void);
    void stop_ec_valves(void);
    
    void init_ref_map();


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
};

#endif
