#include "utils/ec_wrapper.h"

//Power board
PwrStatusMap pow_status_map;
//IMU
ImuStatusMap imu_status_map;
//Force-Torque sensor
FtStatusMap ft_status_map;
// Pump
PumpStatusMap pump_status_map;
PumpReferenceMap pump_reference_map;
std::map<int32_t,ESC_TRJ> pump_trj_map;
// Valve
ValveStatusMap valve_status_map;
ValveReferenceMap valve_reference_map;
std::map<int32_t,ESC_TRJ> valve_trj_map;
// Motor
MotorStatusMap motor_status_map;
MotorReferenceMap motor_reference_map;
std::map<int32_t,ESC_TRJ> motor_trj_map;

using namespace std::chrono;

void adjust_limits(const std::string device_type,
                   const std::vector<std::string>& sdo_limits,
                   std::vector<double>& values)
{
    if(device_type == "motor"){
        auto min_it = std::find(sdo_limits.begin(), sdo_limits.end(), "Min_pos");
        auto max_it = std::find(sdo_limits.begin(), sdo_limits.end(), "Max_pos");

        if (min_it == sdo_limits.end() || max_it == sdo_limits.end())
            return;

        const size_t min_idx = std::distance(sdo_limits.begin(), min_it);
        const size_t max_idx = std::distance(sdo_limits.begin(), max_it);

        constexpr double margin = 0.5 * M_PI / 180.0;

        if (values[max_idx] - values[min_idx] > 2.0 * margin) {
            values[min_idx] += margin;
            values[max_idx] -= margin;
        }

        for (size_t i = 0; i < values.size(); ++i) {
            if (i != min_idx && i != max_idx) {
                values[i] *= 0.95;
            }
        }  
    } 
}

EcWrapper::EcWrapper()
{
    _ec_logger=std::make_shared<EcLogger>();
}

EcWrapper::~EcWrapper()
{
    stop_ec_sys();
}

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

void EcWrapper::get_limits(const int32_t esc_id,
                           const std::string device_type,
                           int   ctrl_mode,
                           std::vector<double> &actual_limit)
{
    actual_limit.clear();

    const auto trj_esc_type_it = esc_trj_map.find(device_type);     
    if (trj_esc_type_it == esc_trj_map.end()) return;           

    std::vector<std::string> sdo_limits;
    std::vector<std::string> ctrl_limits;
    
    for (const auto& [control_mode, trj_info] : trj_esc_type_it->second){
        for (size_t i = 0; i < trj_info.limits.size(); ++i) {
            const auto& limit = trj_info.limits[i];

            if (std::find(sdo_limits.begin(), sdo_limits.end(), limit) == sdo_limits.end()){
                sdo_limits.push_back(limit);
            }

            if(control_mode == ctrl_mode){
                ctrl_limits.push_back(limit);
            }
        }
    }

    if (!sdo_limits.empty()){

        std::vector<double> limits_value(sdo_limits.size());
        read_sdo(esc_id, sdo_limits, limits_value);
        adjust_limits(device_type,sdo_limits,limits_value);

        DPRINTF("Mechanical limits for id: %d\n", esc_id);

        for (size_t i = 0; i < sdo_limits.size(); ++i){

            DPRINTF("%s: %.4f%s",sdo_limits[i].c_str(),
                                 limits_value[i],
                                (i + 1 == sdo_limits.size()) ? "\n" : "   ");

            if (std::find(ctrl_limits.begin(), ctrl_limits.end(), sdo_limits[i]) != ctrl_limits.end()) {
                actual_limit.push_back(limits_value[i]);
            }
        }
    }
}

void EcWrapper::create_ec(EcIface::Ptr &client,EcUtils::EC_CONFIG &ec_cfg)
{
    try{
        ec_cfg=retrieve_ec_cfg();
        _client=_ec_utils->make_ec_iface();
        client=_client;

        _start_devices_vector.clear();
        for (const auto& [id, device_cfg] : _ec_cfg.device_config_map) {
            _start_devices_vector.push_back(id);
        }

        for (const auto& [device_type, trj_cfg] : _ec_cfg.trj_config_map) {
            for (const auto& [id, set_point] : trj_cfg.set_point) {
                const auto cfg_it = _ec_cfg.device_config_map.find(id);
                if (cfg_it == _ec_cfg.device_config_map.end()) continue;             //  unknown configuration for the device (control mode)
                auto ctrl_mode = cfg_it->second.control_mode_type;
        
                const auto trj_esc_type_it = esc_trj_map.find(device_type);     
                if (trj_esc_type_it == esc_trj_map.end()) continue;                   //  unknown device type (motor,valve,pump...)
     
                const auto trj_info_it = trj_esc_type_it->second.find(ctrl_mode);
                if (trj_info_it == trj_esc_type_it->second.end()) continue;           //  unsupported control mode
        
                const auto sp_it = set_point.find(trj_info_it->second.type);
                if (sp_it == set_point.end()) continue;                               //  missing set point

                ESC_TRJ esc_trj{};                                                  
                esc_trj.esc_id = id;

                // set trajectory
                esc_trj.trj1 =  static_cast<double>(sp_it->second);
                esc_trj.trj2 = -static_cast<double>(sp_it->second);
                
                if(trj_info_it->second.type == "position"){
                    const auto homing_it = trj_cfg.homing.find(id);
                    if (homing_it != trj_cfg.homing.end()){
                        esc_trj.trj1 = homing_it->second;
                        esc_trj.trj2 = trj_cfg.trajectory.at(id);
                    }
                }

                // check and set trajectory limit
                std::vector<double> limits_value;
                get_limits(id,device_type,ctrl_mode,limits_value);
                if(!limits_value.empty()){
                    esc_trj.set_trj_limit(limits_value);
                }

                // set actual trajectory
                esc_trj.set_trj = esc_trj.trj1;
                const auto trj_gen_it = trj_cfg.trj_generator.find(id);
                if (trj_gen_it != trj_cfg.trj_generator.end()){
                    esc_trj.general_trj = trj_gen_it->second.at(trj_info_it->second.type);
                }

                if(device_type=="motor"){
                    if(trj_info_it->second.type == "position"){
                        esc_trj.set_zero = esc_trj.trj1;
                    }
                    motor_trj_map[id] = esc_trj;
                }else if(device_type=="valve"){
                    if(trj_info_it->second.type == "position"){
                        esc_trj.set_zero = esc_trj.trj1;
                    }
                    valve_trj_map[id] = esc_trj;
                }else if(device_type=="pump"){
                    pump_trj_map[id] = esc_trj;
                }
            }
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
            if(ec_motors().count(type)>0 || 
               ec_valves().count(type)>0 ||
               ec_pumps().count(type)>0){
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
        if(1){
            // engage brakes OK
            stop_devices=true;
        }
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


template<typename T>
void EcWrapper::read_sdo(const int32_t esc_id,
                         const std::vector<std::string> &sdo_name,
                         std::vector<T> &sdo_read)
{
    if (sdo_read.size() < sdo_name.size()){
        DPRINTF("Problem on %s sdo_read size is less then sdo_name size!\n", __FUNCTION__);
        return;
    }


    WR_SDO wr_sdo;
    RR_SDOS rr_sdo;

    if (!_client->retrieve_rr_sdo(esc_id, sdo_name, wr_sdo, rr_sdo) ||
        rr_sdo.size() != sdo_name.size()){
        DPRINTF("Problem on %s error on retrieve_rr_sdo function!\n", __FUNCTION__);
        return;
    }

    std::unordered_map<std::string, std::size_t> index;
    index.reserve(sdo_name.size());

    for (std::size_t i = 0; i < sdo_name.size(); ++i){
        index.emplace(sdo_name[i], i);
    }

    for (const auto& [reply, value] : rr_sdo){
        auto it = index.find(reply);
        if (it != index.end()){
            T tmp{};
            std::istringstream iss(value);
            if (iss >> tmp){
                sdo_read[it->second] = tmp;
            }
        }
    }
}

template<typename T>
void EcWrapper::write_sdo(const int32_t esc_id,
                          const std::vector<std::string> &sdo_name,
                          const std::vector<T> &sdo_write)
{
    if (sdo_write.size() != sdo_name.size()){
        DPRINTF("Problem on %s sdo_write and sdo_name sizes are different!\n", __FUNCTION__);
        return;
    }

    WR_SDO wr_sdo;
    wr_sdo.reserve(sdo_name.size());

    for (std::size_t i = 0; i < sdo_name.size(); ++i){
        std::ostringstream oss;
        oss << sdo_write[i];
        wr_sdo.emplace_back(sdo_name[i], oss.str());
    }

    if(!_client->set_wr_sdo(esc_id,{},wr_sdo)){
        DPRINTF("Problem on %s error on set_wr_sdo function!\n", __FUNCTION__);
        return;
    }

}

bool EcWrapper::safe_init()
{
    uint8_t count_op_en = 0;
    bool operation_enabled = true;
    const uint8_t max_attempts = 10;

    std::unordered_map<int32_t, bool> motor_is_advrf;
    for (const auto& [esc_id, type, pos] : _slave_info) {
        auto it = ec_motors().find(type);
        if (it != ec_motors().end()) {
            motor_is_advrf[esc_id] = (it->second == "ADVRF_Motor");
        }
    }

    while (count_op_en < max_attempts) {
        _client->read();

        operation_enabled = true;
        _client->get_motor_status(motor_status_map);

        for (const auto& [motor_id, motor_rx_pdo] : motor_status_map){
            if (_ec_cfg.device_config_map.count(motor_id) == 0) continue;

            if (motor_is_advrf.count(motor_id) && 
                motor_is_advrf[motor_id]) continue;

            const auto status_word = std::get<0>(motor_rx_pdo);

            if ((status_word & 0x4) == 0) {
                DPRINTF("Esc id: %d not reached the operational state\n", motor_id);
                operation_enabled = false;
            }
        }

        if (operation_enabled) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ++count_op_en;
    }

    if(!operation_enabled) {
        DPRINTF("Cannot reach operation for all motors requested!\n");
        return false;
    }

    for (const auto &[esc_id, motor_rx_pdo] : motor_status_map){
        auto motor_pos =    std::get<2>(motor_rx_pdo);
        motor_reference_map[esc_id] = {0,motor_pos,0,0,0,0,0,0,0,0,0,0};
        if(_ec_cfg.device_config_map.count(esc_id)>0){
            motor_reference_map[esc_id] = std::make_tuple(  _ec_cfg.device_config_map[esc_id].control_mode_type,  // ctrl_type
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
            if(_ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_POSITION ||
               _ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_IMPEDANCE){
                    if(motor_trj_map.count(esc_id)>0){
                        motor_trj_map[esc_id].start = motor_pos;
                        motor_trj_map[esc_id].set_ref = motor_pos;
                    }
            }
        }
    }

    // init valve reference map 
    _client->get_valve_status(valve_status_map);
    for (const auto &[esc_id, valve_rx_pdo] : valve_status_map){
        auto enc_pos =    std::get<0>(valve_rx_pdo);  
        valve_reference_map[esc_id] = {0,enc_pos,0,0,0,0,0,0,0,0,0,0};          
        if(_ec_cfg.device_config_map.count(esc_id)>0){
            valve_reference_map[esc_id] = std::make_tuple(  0,                                                    // current_ref
                                                            enc_pos,                                              // position_ref
                                                            0,                                                    // force_ref
                                                            _ec_cfg.device_config_map[esc_id].gains[0],           // gain_1
                                                            _ec_cfg.device_config_map[esc_id].gains[1],           // gain_2
                                                            _ec_cfg.device_config_map[esc_id].gains[2],           // gain_3
                                                            _ec_cfg.device_config_map[esc_id].gains[3],           // gain_4
                                                            _ec_cfg.device_config_map[esc_id].gains[4],           // gain_5 
                                                            0,                                                    // fault_ack
                                                            0,                                                    // ts
                                                            0,                                                    // op_idx_aux
                                                            0);                                                   // aux
            if(_ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_POSITION){
                if(valve_trj_map.count(esc_id)>0){
                    valve_trj_map[esc_id].start = enc_pos;
                    valve_trj_map[esc_id].set_ref = enc_pos;
                }
            }
        }
    }

    // init valve reference map
    _client->get_pump_status(pump_status_map);
    for (const auto &[esc_id, pump_rx_pdo] : pump_status_map){
        float pump_target=0.0;
        pump_reference_map[esc_id] = {pump_target,0,0,0,0,0,0,0,0,0};
        if(_ec_cfg.device_config_map.count(esc_id)>0){
            if(_ec_cfg.device_config_map[esc_id].control_mode_type==0xD4){
                pump_target =    std::get<2>(pump_rx_pdo);  //pressure1
            }
            pump_reference_map[esc_id] = std::make_tuple(   pump_target,                                          // Pump_Target
                                                            _ec_cfg.device_config_map[esc_id].gains[0],           // pressure_P_gain
                                                            _ec_cfg.device_config_map[esc_id].gains[1],           // pressure_I_gain
                                                            _ec_cfg.device_config_map[esc_id].gains[2],           // pressure_D_gain
                                                            _ec_cfg.device_config_map[esc_id].gains[3],           // pressure_I_limit
                                                            0,                                                    // fault_ack
                                                            0,                                                    // SolenoidOut
                                                            0,                                                    // ts
                                                            0,                                                    // op_idx_aux
                                                            0);                                                   // aux
            if(_ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_IMPEDANCE){
                if(pump_trj_map.count(esc_id)>0){
                    pump_trj_map[esc_id].start = pump_target;
                    pump_trj_map[esc_id].set_ref = pump_target;
                }
            }
        }
    }

    _client->write();
    return true;
}

void EcWrapper::read_devices_status()
{
    _client->get_motor_status(motor_status_map);
    _client->get_ft_status(ft_status_map);
    _client->get_imu_status(imu_status_map);
    _client->get_pow_status(pow_status_map);
    _client->get_valve_status(valve_status_map);
    _client->get_pump_status(pump_status_map);
}


bool EcWrapper::start_ec_sys(void)
{
    _ec_sys_started=true;
    try{
        
        if(_ec_cfg.protocol == "udp"){
            _client->start_client(_ec_cfg.period_ms); // IMPORTANT: moved here for UDP protocol
        }

        autodetection();

        if(!_start_devices_vector.empty()){
            _ec_sys_started &= start_devices();
        }

#ifdef TEST_LIBRARY
        _ec_sys_started = true;
#endif       
        if(!_ec_sys_started){
            stop_ec_sys();
        }
        else{

            if(_ec_cfg.logging){
                _ec_logger->init_mat_logger(_slave_info);
                _ec_logger->start_mat_logger();
            }
  
            if(_ec_cfg.protocol != "udp"){
                _client->start_client(_ec_cfg.period_ms);
            }

            if(!safe_init()){
                stop_ec_sys();
            } // safe initializaion of the references.
        }

    }catch(std::exception &ex){
        _ec_sys_started=false;
        throw std::runtime_error(ex.what());
    }
    return _ec_sys_started;
}

void EcWrapper::stop_ec_sys(void)
{
    if(_ec_sys_started){
        stop_devices();

        _ec_logger->stop_mat_logger();

        // STOP CLIENT
        _client->stop_client();

        _ec_sys_started=false;
    }
}

void EcWrapper::log_ec_sys()
{
    read_devices_status();

    _ec_logger->log_motor_status(motor_status_map);
    _ec_logger->log_pow_status(pow_status_map);
    _ec_logger->log_ft_status(ft_status_map);
    _ec_logger->log_imu_status(imu_status_map);
    _ec_logger->log_valve_status(valve_status_map);
    _ec_logger->log_pump_status(pump_status_map);


    _ec_logger->log_motor_reference(motor_reference_map); 
    _ec_logger->log_valve_reference(valve_reference_map);
    _ec_logger->log_pump_reference(pump_reference_map);
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
