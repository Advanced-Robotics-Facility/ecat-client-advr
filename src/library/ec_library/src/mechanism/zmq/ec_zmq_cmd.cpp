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
            _client_status.status=ClientStatusEnum::NOT_ALIVE;
            _consoleLog->error("Client in not alive state, please stop the main process!");
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
    
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){
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
                    motor_type_map[esc_id]="ADVRF_MOTOR";
                }
                else if(esc_info["esc_type"]== iit::ecat::SYNAPTICON_v5_0 || esc_info["esc_type"]== iit::ecat::SYNAPTICON_v5_1){
                    motor_type_map[esc_id]="SYNAPTICON_MOTOR";
                }
            }
            
            slave_info.clear();
            slave_info = _slave_info;
            _ec_repl_cmd->set_motor_type_map(motor_type_map);
            
            _client_status.status=ClientStatusEnum::DEVICES_MAPPED;
            
            return true;
        }
        else{
            attemps_cnt++;
        }
    }
    
    if(_slave_info.empty()){
        if(!_fake_slave_info.empty() && 
            _client_status.status!=ClientStatusEnum::NOT_ALIVE){
            _slave_info=_fake_slave_info;
            slave_info = _slave_info;
            return true;
        }     
    }
    
    return false;
}


bool EcZmqCmd::retrieve_all_sdo(uint32_t esc_id,RR_SDOS &rr_sdo)
{
    int attemps_cnt = 0; 
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){
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
                            RR_SDOS &rr_sdo)

{
    int attemps_cnt = 0; 
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){
        std::string msg,rd_sdo_msg;
        auto fault=_ec_repl_cmd->Slave_SDO_cmd(esc_id, 
                                               rd_sdo,
                                               {},  // ignored
                                               rd_sdo_msg);
        
        if(!cmd_error_status(fault, "retrieve_rr_sdo",msg)){
            auto rd_sdo_read = YAML::Load(rd_sdo_msg);
            rr_sdo = rd_sdo_read.as<std::map<std::string, std::string>>();
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
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){
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

bool EcZmqCmd::start_devices(const DST &devices_start)
{
    
    int attemps_cnt = 0; 
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){
        bool devices_started=true;
        for (auto &[device_id ,ctrl_type, gains] : devices_start) {
            std::string msg="";
            auto fault=_ec_repl_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
                                             device_id,
                                             ctrl_type,
                                             gains,
                                             msg);   
            if(cmd_error_status(fault, "start_devices",msg)){
                devices_started &= false; 
                if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
                    break;
                }
            }
        }
        
        if(devices_started){
            _client_status.status=ClientStatusEnum::DEVICES_STARTED;
            _devices_started.clear();
            _devices_started = devices_start;
            return devices_started;
        }
        else{
            attemps_cnt++;
        }
    }

    return false;
}


bool EcZmqCmd::stop_devices()
{
    int attemps_cnt = 0; 
    if(!_devices_started.empty()){
        while(_client_status.status!=ClientStatusEnum::NOT_ALIVE  && attemps_cnt < _max_cmd_attemps){
            bool devices_stopped=true;
            for (auto &[device_id ,ctrl_type, gains] : _devices_started) {
                std::string msg="";
                auto fault=_ec_repl_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                                                  device_id,
                                                  0.0,  // ignored
                                                  {},  // ignored
                                                  msg);
                if(cmd_error_status(fault, "stop_devices",msg)){
                    devices_stopped &= false; 
                    if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
                        break;
                    }
                }
            }
        
            if(devices_stopped){
                _client_status.status=ClientStatusEnum::DEVICES_STOPPED;
                _devices_started.clear();
                return devices_stopped;
            }
            else{
                attemps_cnt++;
            }
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
    while(_client_status.status!=ClientStatusEnum::NOT_ALIVE && attemps_cnt < _max_cmd_attemps){ 
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
    if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
        _consoleLog->error("Client in not alive state, please stop the main process!");
        return;
    }
    else{
        if(_motor_reference_queue.read_available()>0){
            while(_motor_reference_queue.pop(_internal_motor_reference_map))
            {}
            auto fault=_ec_repl_cmd->Motors_PDO_cmd(_internal_motor_reference_map);
            std::string msg="";
            if(!cmd_error_status(fault, "feed_motors",msg)){
            }
        }
    }
}

void EcZmqCmd::feed_valves()
{
    if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
        _consoleLog->error("Client in not alive state, please stop the main process!");
        return;
    }
    else{
        if(_valve_reference_queue.read_available()>0){
            while(_valve_reference_queue.pop(_internal_valve_reference_map))
            {}
        }
    }
}

void EcZmqCmd::feed_pumps()
{
    if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
        _consoleLog->error("Client in not alive state, please stop the main process!");
        return;
    }
    else{
        if(_pump_reference_queue.read_available()>0){
            while(_pump_reference_queue.pop(_internal_pump_reference_map))
            {}
        }
    }
}



//******************************* COMMANDS *****************************************************//





