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

    EcUtils::EC_CONFIG retrieve_ec_cfg();
    void create_ec(EcIface::Ptr &client,EcUtils::EC_CONFIG &ec_cfg);
    void set_start_devices(std::vector<int> motor_start_vector);
    bool start_ec_sys(void);
    void stop_ec_sys(void);

    std::shared_ptr<EcUtils> get_ec_utils();
    void ec_self_sched(std::string thread_name="Thread"); // Scheduling an external thread/process with same policy, cpu and less priority of the ec client.
  
private:
    void autodetection(void);
    void find_devices();
    void prepare_devices();
    bool start_devices(void);
    void stop_devices(void);
    void init_references_maps();


    EcUtils::EC_CONFIG _ec_cfg;
    EcUtils::Ptr _ec_utils;
  
    EcIface::Ptr _client;
    
    DST _start_devices = {};
    PAC _release_brake_cmds = {};
    PAC _engage_brake_cmds = {};
    
    SSI _slave_info;
    std::vector<int> _start_devices_vector;
};

#endif
