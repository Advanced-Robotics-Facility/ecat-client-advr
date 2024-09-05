#include "utils/ec_common_step.h"


using namespace std::chrono;

EcUtils::EC_CONFIG EcCommonStep::retrieve_ec_cfg()
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

void EcCommonStep::create_ec(EcIface::Ptr &client,EcUtils::EC_CONFIG &ec_cfg)
{
    try{
        ec_cfg=retrieve_ec_cfg();
        _client=_ec_utils->make_ec_iface();
        client=_client;

        _motor_start_vector.clear();
        for (auto &[id, pos] : _ec_cfg.homing_position){
            _motor_start_vector.push_back(id);
        }

        _valve_start_vector.clear();
        _valve_start_vector=_ec_cfg.valve_id;

        _start_motor=_ec_cfg.start_motor;
        _start_valve=_ec_cfg.start_valve;

    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
}

std::shared_ptr<EcUtils> EcCommonStep::get_ec_utils()
{
    return _ec_utils;
}
                

void EcCommonStep::autodetection()   
{
    DPRINTF("Try autodetection\n");
    if(_client->retrieve_slaves_info(_slave_info)){   
        if(!_slave_info.empty()){
            DPRINTF("Retrieved slaves\n");
            _motor_id_vector.clear();
            _valve_id_vector.clear();
            for ( auto &[id, type, pos] : _slave_info ) {
                if(ec_motors.count(type)>0){
                    _motor_id_vector.push_back(id);
                }
                else if(type==iit::ecat::HYQ_KNEE){
                    _valve_id_vector.push_back(id);
                }
            }
        }
    }
}

void EcCommonStep::find_motors()   
{
    if(_motor_id_vector.empty()){
        throw std::runtime_error("Got an empty motor id vector for scanning");
    }

    for(int motor_id_index=0;motor_id_index<_motor_id_vector.size();motor_id_index++){
        int motor_id = _motor_id_vector[motor_id_index];
        bool motor_found=false;
        for ( auto &[id, type, pos] : _slave_info ) {
            if(ec_motors.count(type)>0){
                if(id == motor_id){
                    motor_found=true;
                    break;
                }
            }
        }
    
        if(!motor_found){
            throw std::runtime_error("Motor ID: " + std::to_string(motor_id) + " not found");
        }
    }
}

void EcCommonStep::prepare_motors()
{
    _motors_start.clear();
    for(int i=0; i< _motor_id_vector.size();i++)
    {
        int id = _motor_id_vector[i];
        if(_ec_cfg.motor_config_map.count(id)==0){
            throw std::runtime_error("Cannot retrieve a motor configuration for the ID: " + std::to_string(id) + " ,please setup the control mode");
        }
        _motors_start.push_back(std::make_tuple(id,_ec_cfg.motor_config_map[id].control_mode_type,_ec_cfg.motor_config_map[id].gains));
        
        if(_ec_cfg.motor_config_map[id].brake_present){
            // queue release/engage brake commands for all motors 
            _release_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
            _engage_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
        }
    }
}

void EcCommonStep::set_motors_id(std::vector<int> motor_start_vector)
{
    _motor_start_vector=motor_start_vector;
}

bool EcCommonStep::start_ec_motors(void)
{
    bool motor_started=false;
    bool motor_ctrl=motor_started;
    const int max_pdo_aux_cmd_attemps=3;
    int pdo_aux_cmd_attemps=0;

    prepare_motors();
    
    if(!_motors_start.empty())
    {
        // ************************* Start Motors ***********************************//
        DPRINTF("Start motors\n");
        motor_started=_client->start_motors(_motors_start);

        if(motor_started)
        {
            if(!_release_brake_cmds.empty()){
                // ************************* Release brakes ***********************************//
                while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                {
                    pdo_aux_cmd_attemps++;
                    if(!_client->pdo_aux_cmd(_release_brake_cmds))
                    {
                        DPRINTF("Cannot perform the release brake command of the motors\n");
                        pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    }
                    else
                    {
                        std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                        if(_client->pdo_aux_cmd_sts(_release_brake_cmds))
                        {
                            motor_ctrl=true;
                            pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                        }
                    }
                }
                // ************************* Release brakes ***********************************//
            }
            else{
                motor_ctrl=true;
            }
            
        }
        else
        {
            DPRINTF("Motors not started\n");
        }
            
        // ************************* Start Motors ***********************************//
    }
    
    return motor_ctrl;
}


    
void EcCommonStep::stop_ec_motors(void)
{
    bool stop_motors=false;
    const int max_pdo_aux_cmd_attemps=3;
    int pdo_aux_cmd_attemps=0;
    // ************************* Engage brakes BRAKES ***********************************//
    if(!_engage_brake_cmds.empty()){
        pdo_aux_cmd_attemps=0;
        while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
        {
            pdo_aux_cmd_attemps++;
            if(!_client->pdo_aux_cmd(_engage_brake_cmds))
            {
                DPRINTF("Cannot perform the engage brake command of the motors\n");
                pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
            }
            else
            {
                std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                if(_client->pdo_aux_cmd_sts(_engage_brake_cmds))
                {
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    stop_motors=true;
                }
            }
        }
    }
    else{
        stop_motors=true;
    }
    // ************************* Engage brakes ***********************************//

    // ************************* STOP Motors ***********************************//
    if(stop_motors){
        if(!_client->stop_motors()){
            DPRINTF("Not all motors are stopped\n");
        }
        else{
            DPRINTF("All Motors stopped\n");
        }
            
    }
    // ************************* STOP Motors ***********************************//
}

void EcCommonStep::find_valves()
{
    if(_valve_id_vector.empty()){
        throw std::runtime_error("Got an empty valve id vector for scanning");
    }

    for(int valve_id_index=0;valve_id_index<_valve_id_vector.size();valve_id_index++){
        int valve_id = _valve_id_vector[valve_id_index];
        bool valve_found=false;
        for ( auto &[id, type, pos] : _slave_info ) {
            if(type==iit::ecat::HYQ_KNEE){
                if(id == valve_id){
                    valve_found=true;
                    break;
                }
            }
        }
    
        if(!valve_found){
            throw std::runtime_error("Valve ID: " + std::to_string(valve_id) + " not found");
        }
    }
}



void EcCommonStep::set_valves_id(std::vector<int> valve_start_vector)
{
    _valve_start_vector=valve_start_vector;
}

bool EcCommonStep::start_ec_valves(void)
{
    WR_SDO start_valve={std::make_tuple("ctrl_status_cmd","165")};
    bool valve_started=true;
    for(int i=0;i<_valve_id_vector.size();i++){
        int valve_id=_valve_id_vector[i];
        valve_started &=_client->set_wr_sdo(valve_id,{},start_valve);
    }

    if(!valve_started) {
        DPRINTF("Not all valves are started\n");
    }
    else{
        DPRINTF("All Valves started\n");
    }

    return valve_started;
}

void EcCommonStep::stop_ec_valves(void)
{
    WR_SDO stop_valve= {std::make_tuple("ctrl_status_cmd","90")};
    bool valve_stopped=true;
    for(int i=0;i<_valve_id_vector.size();i++){
        int valve_id=_valve_id_vector[i];
        valve_stopped &=_client->set_wr_sdo(valve_id,{},stop_valve);
    }

    if(!valve_stopped) {
        DPRINTF("Not all valves are stopped\n");
    }
    else{
        DPRINTF("All Valves stopped\n");
    }
}

bool EcCommonStep::start_ec_sys(void)
{
    bool ec_sts_started=true;
    try{
        autodetection();

        if(_start_motor){
            if(!_motor_start_vector.empty()){
                _motor_id_vector=_motor_start_vector;
                find_motors();
            }

            ec_sts_started &= start_ec_motors();
        }
        
        if(_start_valve){

            if(!_valve_start_vector.empty()){
                _valve_id_vector=_valve_start_vector;
                find_valves();
            }

            ec_sts_started &= start_ec_valves();
        }

#ifdef TEST_LIBRARY
        ec_sts_started = true;
#endif       
        if(ec_sts_started){
            _client->start_client(_ec_cfg.period_ms,_ec_cfg.logging);
        }
    }catch(std::exception &ex){
        ec_sts_started=false;
        throw std::runtime_error(ex.what());
    }
    return ec_sts_started;
}

void EcCommonStep::stop_ec_sys(void)
{
    if(_start_motor){
        stop_ec_motors();
    }

    if(_start_valve){
        stop_ec_valves();
    }

    // STOP CLIENT
    if(_client->is_client_alive()){
        _client->stop_client();
    }
}

void EcCommonStep::telemetry()
{
    _client->get_pow_status(_pow_status_map);
    _client->get_imu_status(_imu_status_map);
    _client->get_pump_status(_pump_status_map);
    _client->get_valve_status(_valve_status_map);
    _client->get_motors_status(_motors_status_map);
    
    if(!_print_telemetry){
        return;
    }

    //******************* Power Board Telemetry ********
    for (const auto &[esc_id, pow_rx_pdo] : _pow_status_map){
        auto v_batt =        std::get<0>(pow_rx_pdo);
        auto v_load =        std::get<1>(pow_rx_pdo);
        auto i_load =        std::get<2>(pow_rx_pdo);
        auto temp_pcb =      std::get<3>(pow_rx_pdo);
        auto temp_heatsink = std::get<4>(pow_rx_pdo);
        auto temp_batt =     std::get<5>(pow_rx_pdo);
 
        DPRINTF("POW ID: [%d], VBATT: [%f], VLOAD: [%f], ILOAD: [%f]\n",esc_id,v_batt,v_load,i_load);
        DPRINTF("POW ID: [%d], Temp_pcb: [%f], Temp_heatsink: [%f], Temp_batt: [%f]\n",esc_id,temp_pcb,temp_heatsink,temp_batt);
    }
    //******************* Power Board Telemetry ********

    //******************* IMU Telemetry ********

    for (const auto &[esc_id, imu_rx_pdo] : _imu_status_map){
        auto x_rate = std::get<0>(imu_rx_pdo);
        auto y_rate = std::get<1>(imu_rx_pdo);
        auto z_rate = std::get<2>(imu_rx_pdo);
        auto x_acc =  std::get<3>(imu_rx_pdo);
        auto y_acc =  std::get<4>(imu_rx_pdo);
        auto z_acc =  std::get<5>(imu_rx_pdo);
        auto x_quat = std::get<6>(imu_rx_pdo);
        auto y_quat = std::get<7>(imu_rx_pdo);
        auto z_quat = std::get<8>(imu_rx_pdo);
        auto w_quat = std::get<9>(imu_rx_pdo);

        DPRINTF("IMU ID: [%d], X_RATE: [%f], Y_RATE: [%f], Z_RATE: [%f]\n",esc_id,x_rate,y_rate,z_rate);
        DPRINTF("IMU ID: [%d], X_ACC: [%f], Y_ACC: [%f], Z_ACC: [%f]\n",esc_id,x_acc,y_acc,z_acc);
        DPRINTF("IMU ID: [%d], X_QUAT: [%f], Y_QUAT: [%f], Z_QUAT: [%f], W_QUAT: [%f]\n",esc_id,x_quat,y_quat,z_quat,w_quat);
    }
    //******************* IMU Telemetry ********

    //******************* Pump Telemetry ********
    for (const auto &[esc_id, pump_rx_pdo] : _pump_status_map){
        DPRINTF("PUMP ID: [%d], Pressure: [%hhu] \n",esc_id,std::get<0>(pump_rx_pdo));
    }
    //******************* Pump Telemetry ********

    //******************* Valve Telemetry ********
    for (const auto &[esc_id, valve_rx_pdo] : _valve_status_map){
        auto encoder_position = std::get<0>(valve_rx_pdo);
        auto tor_valve = std::get<1>(valve_rx_pdo);
        auto pressure1 = std::get<2>(valve_rx_pdo);
        auto pressure2 = std::get<3>(valve_rx_pdo);
        auto temperature = std::get<4>(valve_rx_pdo);
            
        DPRINTF("VALVE ID: [%d], Encoder pos: [%f], Torque: [%f]\n",esc_id,encoder_position,tor_valve);
        DPRINTF("VALVE ID: [%d], Press1: [%f], Press2: [%f],Temp: [%f]\n",esc_id,pressure1,pressure2,temperature);
    }
    //******************* Valve Telemetry ********


     //******************* Motor Telemetry **************
    for (const auto &[esc_id, motor_rx_pdo] : _motors_status_map)
    {
        auto link_pos =     std::get<0>(motor_rx_pdo);
        auto motor_pos =    std::get<1>(motor_rx_pdo);
        auto link_vel =     std::get<2>(motor_rx_pdo);
        auto motor_vel =    std::get<3>(motor_rx_pdo);
        auto torque =       std::get<4>(motor_rx_pdo);
        auto motor_temp =   std::get<5>(motor_rx_pdo);
        auto board_temp =   std::get<6>(motor_rx_pdo);
        auto fault =        std::get<7>(motor_rx_pdo);
        auto rtt =          std::get<8>(motor_rx_pdo);
        auto op_idx_ack =   std::get<9>(motor_rx_pdo);       
        auto aux =          std::get<10>(motor_rx_pdo);
        auto cmd_aux_sts =  std::get<11>(motor_rx_pdo);

        // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.
        auto brake_sts = cmd_aux_sts & 3; // 00 unknown
                                          // 01 release brake
                                          // 10 enganged brake
                                          // 11 error
        auto led_sts = (cmd_aux_sts & 4) / 4; // 1 or 0 LED  ON/OFF


        DPRINTF("MOTOR ID: [%d], Link pos: [%f], Motor pos: [%f]\n",esc_id,link_pos,motor_pos);

    }
    //******************* Motor Telemetry **************
}
