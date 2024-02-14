#include <cassert>
#include <tuple>

#include "common/ec_cmd.h"
#include <yaml-cpp/yaml.h>
#include <chrono>

EcCmd::EcCmd(std::string protocol,std::string host_address,uint32_t host_port)
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
    
    _ec_zmq_cmd = std::make_shared<EcZmqCmd>(zmq_uri,timeout_ms);
    
    _consoleLog->info("ZMQ_URI: {}",zmq_uri);

}

EcCmd::~EcCmd()
{

}

bool EcCmd::cmd_error_status(EcZmqFault fault, std::string op, std::string &msg)
{
    bool cmd_error=false;
    msg.clear();
    
    if(fault.get_type() != EC_ZMQ_CMD_STATUS::OK){
        msg = "commad: " + op + 
              " failed: " +  fault.get_info() + 
              " recovery: " + fault.get_recovery_info();
        cmd_error=true;
        
        _consoleLog->error(msg);
        
        if(fault.get_type() == EC_ZMQ_CMD_STATUS::TIMEOUT){
            _client_alive=false;
            _consoleLog->error("Client not alive, please stop the main process!");
        }
    }
    
    return cmd_error;
}


bool EcCmd::retrieve_slaves_info(SSI &slave_info)
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
        
        auto fault=_ec_zmq_cmd->Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR,
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
            _ec_zmq_cmd->set_motor_type_map(motor_type_map);
            
            return true;
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}


bool EcCmd::retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo)
{
    return false;
}

bool EcCmd::retrieve_rr_sdo(uint32_t esc_id,
                            const RD_SDO &rd_sdo, 
                            const WR_SDO &wr_sdo,
                            RR_SDO &rr_sdo)

{
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string msg,rd_sdo_msg;
        auto fault=_ec_zmq_cmd->Slave_SDO_cmd(esc_id, 
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

bool EcCmd::set_wr_sdo(uint32_t esc_id,
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
        auto fault=_ec_zmq_cmd->Slave_SDO_cmd(esc_id, 
                                              {},  // ignored
                                              wr_sdo_map,
                                              wd_sdo_msg);
        
        if(!cmd_error_status(fault, "retrieve_rr_sdo",msg)){
            return true;
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}

bool EcCmd::start_motors(const MST &motors_start)
{
    
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        bool motors_started=true;
        for (auto &[motor_id ,ctrl_type, gains] : motors_start) {
            
            if(!_client_alive){
                return false;
            }
            
            std::string msg="";
            auto fault=_ec_zmq_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
                                             motor_id,
                                             ctrl_type,
                                             gains,
                                             msg);   
            if(cmd_error_status(fault, "start_motors",msg)){
                motors_started &= false; 
            }
        }
        
        if(motors_started){
            return motors_started;
        }
        else{
            attemps_cnt++;
        }
        
    }

    return false;
}


bool EcCmd::stop_motors()
{
    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        bool motors_stopped=true;
        for ( auto &[esc_id, type, pos] : _slave_info ) {
            
            if(!_client_alive){
                return false;
            }
            
            std::string msg="";
            auto fault=_ec_zmq_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                                             esc_id,
                                             0.0,  // ignored
                                             {},  // ignored
                                             msg);
            if(cmd_error_status(fault, "stop_motors",msg))
            {
                motors_stopped &= false; 
            }
        }
        
        if(motors_stopped){
            return motors_stopped;
        }
        else{
            attemps_cnt++;
        }
    }
    return false;
}

bool EcCmd::pdo_aux_cmd(const PAC & pac)
{
    std::vector<EcZmqCmd::aux_cmd_message_t> aux_cmds;

    for ( auto &[esc_id, cmd_type ] : pac ) {
        EcZmqCmd::aux_cmd_message_t aux_cmd;

        // prepare message for releasing or engaging the brake
        aux_cmd.board_id = esc_id;
        aux_cmd.type=static_cast<iit::advr::PDOs_aux_cmd_Aux_cmd_Type>(cmd_type);
        aux_cmds.push_back(aux_cmd);
    }

    int attemps_cnt = 0; 
    while(_client_alive && attemps_cnt < _max_cmd_attemps){ 
        std::string msg="";
        
        // send command
        auto fault=_ec_zmq_cmd->PDOs_aux_cmd(aux_cmds,msg);
        
        if(!cmd_error_status(fault, "pdo_aux_cmd",msg)){
            return true; 
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}

void EcCmd::feed_motors()
{
   auto start_cmd_all= std::chrono::steady_clock::now();
    if(!_client_alive){
        _consoleLog->error("Client not alive, please stop the main process!");
        return;
    }
    else{
            
        pthread_mutex_lock(&_mutex_motor_reference);
        if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE){
            if(!_motors_references.empty()){
                auto start_cmd= std::chrono::steady_clock::now();
                auto fault=_ec_zmq_cmd->Motors_PDO_cmd(_motors_references);
                if(fault.get_type() == EC_ZMQ_CMD_STATUS::TIMEOUT){
                    _consoleLog->error("Client not alive, please stop the main process!");
                    _client_alive=false;
                }
                _ec_logger->log_motors_ref(_motors_references);
                
                auto end_cmd= std::chrono::steady_clock::now();
                auto time_elapsed_us= std::chrono::duration_cast<std::chrono::microseconds>(end_cmd-start_cmd);
                if(time_elapsed_us > std::chrono::microseconds(500)){
                _consoleLog->info("commad executed: {}",time_elapsed_us.count());
                }
                
            }
            else{
                _consoleLog->error("Got empty motors references structure");
            }
        }
        pthread_mutex_unlock(&_mutex_motor_reference);
    }
    
    auto end_cmd_all= std::chrono::steady_clock::now();
    auto time_elapsed_all_us= std::chrono::duration_cast<std::chrono::microseconds>(end_cmd_all-start_cmd_all);
    if(time_elapsed_all_us > std::chrono::microseconds(500)){
    _consoleLog->info("all commad executed: {}",time_elapsed_all_us.count());
    }
}

//******************************* COMMANDS *****************************************************//





