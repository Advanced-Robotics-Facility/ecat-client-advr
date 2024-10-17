#include "utils/ec_wrapper.h"

//Power board
PwrStatusMap pow_status_map;
//IMU
ImuStatusMap imu_status_map;
// Pump
PumpStatusMap pump_status_map;
PumpReferenceMap pumps_ref;
// Valve
ValveStatusMap valve_status_map;
ValveReferenceMap valves_ref;
// Motor
MotorStatusMap motors_status_map;
MotorReferenceMap motors_ref;


using namespace std::chrono;

EcUtils::EC_CONFIG EcWrapper::retrieve_ec_cfg()
{
    try{
        _ec_utils=std::make_shared<EcUtils>();
        _ec_cfg = _ec_utils->get_ec_cfg();
    }catch(std::exception &ex){
        DPRINTF("Error on ec client config file \n");
        throw std::runtime_error(ex.what());
    }
    return _ec_cfg;
}

void EcWrapper::create_ec(EcIface::Ptr &client,EcUtils::EC_CONFIG &ec_cfg)
{
    try{
        ec_cfg=retrieve_ec_cfg();
        _client=_ec_utils->make_ec_iface();
        client=_client;
        _start_devices_vector.clear();
        for(const auto&[id,device_cfg]:_ec_cfg.device_config_map){
            _start_devices_vector.push_back(id);
        }
    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
}

std::shared_ptr<EcUtils> EcWrapper::get_ec_utils()
{
    return _ec_utils;
}

void EcWrapper::set_start_devices(std::vector<int> start_devices_vector)
{
    _start_devices_vector=start_devices_vector;
}
                

void EcWrapper::autodetection()   
{
    if(_client->retrieve_slaves_info(_slave_info)){   
        if(!_slave_info.empty()){
            DPRINTF("Retrieved slaves\n");
        }
    }
}


void EcWrapper::find_devices()   
{
    for(const auto& device_id:_start_devices_vector){
        bool device_found=false;
        for ( auto &[id, type, pos] : _slave_info ) {
            if(ec_motors.count(type)>0 || ec_valves.count(type)){
                if(id == device_id){
                    device_found=true;
                    break;
                }
            }
        }
        if(!device_found){
            throw std::runtime_error("Device ID: " + std::to_string(device_id) + " not found");
        }
    }
}

void EcWrapper::prepare_devices()
{
    for(const auto& id:_start_devices_vector){
        if(_ec_cfg.device_config_map.count(id)==0){
            throw std::runtime_error("Cannot retrieve a device configuration for the ID: " + std::to_string(id) + " ,please setup the control mode");
        }
        _start_devices.push_back(std::make_tuple(id,_ec_cfg.device_config_map[id].control_mode_type,_ec_cfg.device_config_map[id].gains));
        
        if(_ec_cfg.device_config_map[id].brake_present){
            // queue release/engage brake commands for all motors 
            _release_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
            _engage_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
        }
    }
}

bool EcWrapper::start_devices(void)
{
    bool devices_started=false;

    find_devices();

    prepare_devices();
    
    if(!_start_devices.empty()){
        devices_started=_client->start_devices(_start_devices);

        if(devices_started){
            //release brakes 
            DPRINTF("Devices started\n");
        }
        else{
            DPRINTF("Problem of devices starting phase\n");
        }
    }
    
    return devices_started;
}


    
void EcWrapper::stop_devices(void)
{
    bool stop_devices=false;
    if(!_engage_brake_cmds.empty()){
        // engage brakes
    }
    else{
        stop_devices=true;
    }

    if(stop_devices){
        if(!_client->stop_devices()){
            DPRINTF("Problem of devices stopping phase\n");
        }
        else{
            DPRINTF("Devices stopped\n");
        }
            
    }
}

void EcWrapper::init_references_maps()
{
    
    // init motor reference map 
    _client->get_motors_status(motors_status_map);
    for (const auto &[esc_id, motor_rx_pdo] : motors_status_map){
        auto motor_pos =    std::get<1>(motor_rx_pdo);
        motors_ref[esc_id] = std::make_tuple(   _ec_cfg.device_config_map[esc_id].control_mode_type,  // ctrl_type
                                                motor_pos,                                            // pos_ref
                                                0.0,                                                  // vel_ref
                                                0.0,                                                  // tor_ref
                                                _ec_cfg.device_config_map[esc_id].gains[0],           // gain_1
                                                _ec_cfg.device_config_map[esc_id].gains[1],           // gain_2
                                                _ec_cfg.device_config_map[esc_id].gains[2],           // gain_3
                                                _ec_cfg.device_config_map[esc_id].gains[3],           // gain_4
                                                _ec_cfg.device_config_map[esc_id].gains[4],           // gain_5
                                                1,                                                    // op means NO_OP
                                                0,                                                    // idx
                                                0                                                     // aux
                                            );
    }

    // init valve reference map 
    _client->get_valve_status(valve_status_map);
    for (const auto &[esc_id, valve_rx_pdo] : valve_status_map){
        auto enc_pos =    std::get<0>(valve_rx_pdo);
        valves_ref[esc_id] = std::make_tuple(0, enc_pos, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    // init valve reference map
    _client->get_pump_status(pump_status_map);
    for (const auto &[esc_id, pump_rx_pdo] : pump_status_map){
        pumps_ref[esc_id] = std::make_tuple(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
}


bool EcWrapper::start_ec_sys(void)
{
    bool ec_sts_started=true;
    try{
        _client->start_client(_ec_cfg.period_ms,_ec_cfg.logging); // IMPORTANT: moved here for UDP protocol
        
        autodetection();

        if(!_start_devices_vector.empty()){
            ec_sts_started &= start_devices();
        }


#ifdef TEST_LIBRARY
        ec_sts_started = true;
#endif       
        if(ec_sts_started){
            _client->read();
            init_references_maps();
        }
        else{
            stop_ec_sys();
        }
    }catch(std::exception &ex){
        ec_sts_started=false;
        throw std::runtime_error(ex.what());
    }
    return ec_sts_started;
}

void EcWrapper::stop_ec_sys(void)
{
    stop_devices();

    // STOP CLIENT
    _client->stop_client();

}


void EcWrapper::ec_self_sched(std::string thread_name)
{
    if(_ec_cfg.protocol == "iddp"){
        #if defined(PREEMPT_RT) || defined(__COBALT__)
            DPRINTF("Real-time process....\n");
        #endif
    }

    if(_client != nullptr){
        auto client_thread_info = _client->get_client_thread_info();
        int priority=std::max(0,client_thread_info.priority-1);
        struct sched_param  schedparam;
        schedparam.sched_priority = std::max(0,priority);
        int ret= pthread_setschedparam ( pthread_self(), client_thread_info.policy, &schedparam );
        if (ret < 0){
            throw std::runtime_error("fatal error on ec_self_sched, got an error on pthread_setschedparam");
        }
        
        if(client_thread_info.cpu>=0){
            cpu_set_t cpu_set;
            CPU_ZERO ( &cpu_set );
            CPU_SET ( client_thread_info.cpu,&cpu_set );
            pthread_setaffinity_np( pthread_self(), sizeof ( cpu_set ), &cpu_set );
        }
        if (ret < 0){
            throw std::runtime_error("fatal error on ec_self_sched, got an error on pthread_setaffinity_np");
        }
        
        DPRINTF("%s initialized, ",thread_name.c_str());
        DPRINTF("id: %ld cpu: %d, priority %d\n",pthread_self(),sched_getcpu(),priority);
    }
    else{
        throw std::runtime_error("fatal error on ec_self_sched function got an empty client!");
    }
    
}