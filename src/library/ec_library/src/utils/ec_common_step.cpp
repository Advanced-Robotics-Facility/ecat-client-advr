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

void EcCommonStep::find_motors(std::vector<int> motor_id_vector)   
{
    if(motor_id_vector.empty()){
        throw std::runtime_error("Got an empty motor id vector for scanning");
    }
    else{
        _motor_id_vector=motor_id_vector;
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

bool EcCommonStep::start_ec_motors(std::vector<int> motor_id_vector)
{
    try{
        find_motors(motor_id_vector);
        bool ret=start_ec_motors();
        return ret;
    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
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

void EcCommonStep::find_valves(std::vector<int> valve_id_vector)
{
    if(valve_id_vector.empty()){
        throw std::runtime_error("Got an empty valve id vector for scanning");
    }
    else{
        _valve_id_vector=valve_id_vector;
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



bool EcCommonStep::start_ec_valves(std::vector<int> valve_id_vector)
{
    try{
        find_valves(valve_id_vector);
        bool ret=start_ec_valves();
        return ret;
    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
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

void EcCommonStep::start_ec(void)
{
    try{
        _client->start_client(_ec_cfg.period_ms,_ec_cfg.logging);
    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
}

void EcCommonStep::stop_ec(void)
{
    // STOP CLIENT
    if(_client->is_client_alive()){
        _client->stop_client();
    }
}
