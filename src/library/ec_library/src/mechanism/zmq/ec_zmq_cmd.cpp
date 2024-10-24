#include <cassert>
#include <tuple>

#include "mechanism/zmq/ec_zmq_cmd.h"
#include <yaml-cpp/yaml.h>
#include <chrono>

EcZmqCmd::EcZmqCmd(std::string protocol,std::string host_address,uint32_t host_port)
{

    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    std::string host_port_cmd = std::to_string(host_port+555);
    
    // zmq setup
    std::string zmq_uri = protocol+"://" + host_address + ":"+host_port_cmd;
    int timeout_ms = 500;  // 0.5 secs
    
    _ec_repl_cmd = std::make_shared<EcReplCmd>(zmq_uri,timeout_ms);
    
    _consoleLog->info("ZMQ_URI: {}",zmq_uri);

}

EcZmqCmd::~EcZmqCmd()
{

}

bool EcZmqCmd::cmd_error_status(EcReplFault fault, std::string op, std::string &msg)
{
    bool cmd_error=false;
    msg.clear();
    
    if(fault.get_type() != EC_REPL_CMD_STATUS::OK){
        msg = "commad: " + op + 
              " failed, " +  fault.get_info() + 
              " recovery: " + fault.get_recovery_info();
        cmd_error=true;
        
        _consoleLog->error(msg);
        
        if(fault.get_type() == EC_REPL_CMD_STATUS::TIMEOUT){
            _client_alive=false;
            _consoleLog->error("Client not alive, please stop the main process!");
        }
    }
    
    return cmd_error;
}


bool EcZmqCmd::retrieve_slaves_info(SSI &slave_info)
{
    int attemps_cnt = 0; 
    
    if(!_slave_info.empty())
    {
        slave_info=_slave_info;
        return true;
    }
    
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string slave_descr_info,msg="";
        
        std::map<std::string,std::string> args;
        
        auto fault=_ec_repl_cmd->Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR,
                                                args,
                                                slave_descr_info);
        
        if(!cmd_error_status(fault, "retrieve_slaves_info",msg)){

            _slave_info.clear();
            
            // load slave descr as yaml
            YAML::Node slaves_info = YAML::Load(slave_descr_info);
            
            auto slave_map = slaves_info.as<std::map<int, std::map<std::string, int>>>();
            std::map<int32_t,std::string> motor_type_map;
            for ( auto const &item : slave_map ) {
                auto esc_id = item.first;
                auto esc_info = item.second;
                _slave_info.push_back(std::make_tuple(esc_id,esc_info["esc_type"],esc_info["position"]));
                if(esc_info["esc_type"]== iit::ecat::CENT_AC || esc_info["esc_type"]==iit::ecat::LO_PWR_DC_MC){
                    motor_type_map[esc_id]="HHCM_MOTOR";
                }
                else if(esc_info["esc_type"]== iit::ecat::CIRCULO9){
                    motor_type_map[esc_id]="CIRCULO9_MOTOR";
                }
                else if(esc_info["esc_type"]== iit::ecat::AMC_FLEXPRO){
                    motor_type_map[esc_id]="AMC_FLEXPRO_MOTOR";
                }
            }
            
            slave_info.clear();
            slave_info = _slave_info;
            _ec_repl_cmd->set_motor_type_map(motor_type_map);
            
            _client_status=ClientStatus::MOTORS_MAPPED;
            
            _ec_logger->init_mat_logger(_slave_info);
            return true;
        }
        else{
            attemps_cnt++;
        }
    }
    
    if(_slave_info.empty()){
        if(!_fake_slave_info.empty()){
            _slave_info=_fake_slave_info;
            slave_info = _slave_info;
            _ec_logger->init_mat_logger(_slave_info);
            return true;
        }     
    }
    
    return false;
}


bool EcZmqCmd::retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo)
{
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string msg,rd_all_sdo_name;
        auto fault=_ec_repl_cmd->Slave_SDO_info(iit::advr::Slave_SDO_info_Type::Slave_SDO_info_Type_SDO_NAME, 
                                                esc_id,
                                                rd_all_sdo_name);
        
        if(!cmd_error_status(fault, "retrieve_all_sdo",msg)){
            auto rd_sdo_name_read = YAML::Load(rd_all_sdo_name);
            RD_SDO rd_sdo = rd_sdo_name_read.as<std::vector<std::string>>();
            WR_SDO wr_sdo;
            bool ret=retrieve_rr_sdo(esc_id,rd_sdo,wr_sdo,rr_sdo);
            return ret;
        }
        else{
            attemps_cnt++;
        }
        
    }
    return false;
}

bool EcZmqCmd::retrieve_rr_sdo(uint32_t esc_id,
                            const RD_SDO &rd_sdo, 
                            const WR_SDO &wr_sdo,
                            RR_SDO &rr_sdo)

{
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string msg,rd_sdo_msg;
        auto fault=_ec_repl_cmd->Slave_SDO_cmd(esc_id, 
                                               rd_sdo,
                                               {},  // ignored
                                               rd_sdo_msg);
        
        if(!cmd_error_status(fault, "retrieve_rr_sdo",msg)){
            auto rd_sdo_read = YAML::Load(rd_sdo_msg);
            rr_sdo = rd_sdo_read.as<std::map<std::string, float>>();
            
            return true;
        }
        else{
            attemps_cnt++;
        }
        
    }
    return false;
}

bool EcZmqCmd::set_wr_sdo(uint32_t esc_id,
                       const RD_SDO &rd_sdo,
                       const WR_SDO &wr_sdo)

{
    std::map<std::string ,std::string> wr_sdo_map;
    for (auto &[sdo_name ,value] : wr_sdo) {
        wr_sdo_map[sdo_name]=value;
    }
        
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string msg,wd_sdo_msg;
        auto fault=_ec_repl_cmd->Slave_SDO_cmd(esc_id, 
                                              {},  // ignored
                                              wr_sdo_map,
                                              wd_sdo_msg);
        
        if(!cmd_error_status(fault, "set_wr_sdo",msg)){
            return true;
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}

bool EcZmqCmd::start_motors(const MST &motors_start)
{
    
    int attemps_cnt = 0; 
    // stop references
    _motor_ref_flags=RefFlags::FLAG_NONE;
    _valve_ref_flags=RefFlags::FLAG_NONE;
    
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        bool motors_started=true;
        for (auto &[motor_id ,ctrl_type, gains] : motors_start) {
            
            if(!_client_alive){
                return false;
            }
            
            std::string msg="";
            auto fault=_ec_repl_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
                                             motor_id,
                                             ctrl_type,
                                             gains,
                                             msg);   
            if(cmd_error_status(fault, "start_motors",msg)){
                motors_started &= false; 
            }
        }
        
        if(motors_started){
            _client_status=ClientStatus::MOTORS_STARTED;
            return motors_started;
        }
        else{
            attemps_cnt++;
        }
        
    }

    return false;
}


bool EcZmqCmd::stop_motors()
{
    int attemps_cnt = 0; 
    // stop references
    _motor_ref_flags=RefFlags::FLAG_NONE;
    _valve_ref_flags=RefFlags::FLAG_NONE;
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        bool motors_stopped=true;
        for ( auto &[esc_id, type, pos] : _slave_info ) {
            
            if(!_client_alive){
                return false;
            }
            if(ec_motors.count(esc_id)>0){
                std::string msg="";
                auto fault=_ec_repl_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                                                esc_id,
                                                0.0,  // ignored
                                                {},  // ignored
                                                msg);
                if(cmd_error_status(fault, "stop_motors",msg))
                {
                    motors_stopped &= false; 
                }
            }
        }
        
        if(motors_stopped){
            _client_status=ClientStatus::MOTORS_STOPPED;
            return motors_stopped;
        }
        else{
            attemps_cnt++;
        }
    }
    return false;
}

bool EcZmqCmd::pdo_aux_cmd(const PAC & pac)
{
    std::vector<EcReplCmd::aux_cmd_message_t> aux_cmds;

    for ( auto &[esc_id, cmd_type ] : pac ) {
        EcReplCmd::aux_cmd_message_t aux_cmd;

        // prepare message for releasing or engaging the brake
        aux_cmd.board_id = esc_id;
        aux_cmd.type=static_cast<iit::advr::PDOs_aux_cmd_Aux_cmd_Type>(cmd_type);
        aux_cmds.push_back(aux_cmd);
    }

    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){ 
        std::string msg="";
        
        // send command
        auto fault=_ec_repl_cmd->PDOs_aux_cmd(aux_cmds,msg);
        
        if(!cmd_error_status(fault, "pdo_aux_cmd",msg)){
            return true; 
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}

void EcZmqCmd::send_pdo()
{
    feed_motors(); 
    feed_valves();
    feed_pumps();
}

void EcZmqCmd::feed_motors()
{
    if(!_client_alive){
        _consoleLog->error("Client not alive, please stop the main process!");
        return;
    }
    else{
        if(_motor_ref_flags!=RefFlags::FLAG_NONE){

            pthread_mutex_lock(&_mutex_motor_reference);
            _motors_references_cmd=_motors_references;
            _ec_logger->log_motors_ref(_motors_references_cmd);
            pthread_mutex_unlock(&_mutex_motor_reference);
            
            if(!_motors_references_cmd.empty()){
                std::string msg="";
                auto fault=_ec_repl_cmd->Motors_PDO_cmd(_motors_references_cmd);
/*                 if(!cmd_error_status(fault, "feed_motors",msg)){
                    // ...continue if not in timeout
                } */
            }
            else{
                _consoleLog->error("Got empty motors references structure");
            }
        }
    }
}

void EcZmqCmd::feed_valves()
{
    if(!_client_alive){
        _consoleLog->error("Client not alive, please stop the main process!");
        return;
    }
    else{
        if(_valve_ref_flags!=RefFlags::FLAG_NONE){

            pthread_mutex_lock(&_mutex_valve_reference);
            _valves_references_cmd=_valves_references;
            _ec_logger->log_valve_ref(_valves_references_cmd);   
            pthread_mutex_unlock(&_mutex_valve_reference);

            if(!_valves_references_cmd.empty()){
//                 auto fault=_ec_repl_cmd->Motors_PDO_cmd(_motors_references);
//                 if(fault.get_type() == EC_REPL_CMD_STATUS::TIMEOUT){
//                     _consoleLog->error("Client not alive, please stop the main process!");
//                     _client_alive=false;
//                 }
            }
            else{
                _consoleLog->error("Got empty valves references structure");
            }
        }
    }
}

void EcZmqCmd::feed_pumps()
{
    if(!_client_alive){
        _consoleLog->error("Client not alive, please stop the main process!");
        return;
    }
    else{
        if(_pump_ref_flags!=RefFlags::FLAG_NONE){
            pthread_mutex_lock(&_mutex_pump_reference);
            _pumps_references_cmd=_pumps_references;
            _ec_logger->log_pump_ref(_pumps_references_cmd); 
            pthread_mutex_unlock(&_mutex_pump_reference);
            if(!_pumps_references_cmd.empty()){
//                 auto fault=_ec_repl_cmd->Motors_PDO_cmd(_motors_references);
//                 if(fault.get_type() == EC_REPL_CMD_STATUS::TIMEOUT){
//                     _consoleLog->error("Client not alive, please stop the main process!");
//                     _client_alive=false;
//                 }
            }
            else{
                _consoleLog->error("Got empty pumps references structure");
            }
        }
    }
}



//******************************* COMMANDS *****************************************************//





