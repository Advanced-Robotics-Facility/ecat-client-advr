#include <cassert>
#include <tuple>

#include "ec_cmd.h"
#include <yaml-cpp/yaml.h>

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
    int timeout_ms = 1000;  // 1 secs
    
    _ec_zmq_cmd = std::make_shared<EcZmqCmd>(zmq_uri,timeout_ms);
    _client_alive=true;
    
    _consoleLog=spdlog::get("console");
    if(!_consoleLog)
    {
        createLogger("console","client");
        _consoleLog=spdlog::get("console");
    }
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
    while(_client_alive && attemps_cnt < _max_cmd_attemps){
        std::string slave_descr_info,msg="";
        
        std::map<std::string,std::string> args;
        
        _ec_zmq_cmd->Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR,
                                    args,
                                    slave_descr_info);
        
        if(!cmd_error_status(_ec_zmq_cmd->get_fault(), "retrieve_slaves_info",msg)){

            _slave_info.clear();
            
            // load slave descr as yaml
            YAML::Node slaves_info = YAML::Load(slave_descr_info);
            
            auto slave_map = slaves_info.as<std::map<int, std::map<std::string, int>>>();
            for ( auto const &item : slave_map ) {
                auto esc_id = item.first;
                auto esc_info = item.second;
                _slave_info.push_back(std::make_tuple(esc_id,esc_info["esc_type"],esc_info["position"]));
            }
            
            slave_info.clear();
            slave_info = _slave_info;
            
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
        _ec_zmq_cmd->Slave_SDO_cmd(esc_id, 
                                   rd_sdo,
                                   {},  // ignored
                                   rd_sdo_msg);
        
        if(!cmd_error_status(_ec_zmq_cmd->get_fault(), "retrieve_rr_sdo",msg)){
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
        _ec_zmq_cmd->Slave_SDO_cmd(esc_id, 
                                   {},  // ignored
                                   wr_sdo_map,
                                   wd_sdo_msg);
        
        if(!cmd_error_status(_ec_zmq_cmd->get_fault(), "retrieve_rr_sdo",msg)){
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
            std::string msg="";
            _ec_zmq_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
                                motor_id,
                                ctrl_type,
                                gains,
                                msg);   
            if(cmd_error_status(_ec_zmq_cmd->get_fault(), "start_motors",msg)){
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
            std::string msg="";
            _ec_zmq_cmd->Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                                esc_id,
                                0.0,  // ignored
                                {},  // ignored
                                msg);
            if(cmd_error_status(_ec_zmq_cmd->get_fault(), "stop_motors",msg))
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
        _ec_zmq_cmd->PDOs_aux_cmd(aux_cmds,msg);
        
        if(!cmd_error_status(_ec_zmq_cmd->get_fault(), "pdo_aux_cmd",msg)){
            return true; 
        }
        else{
            attemps_cnt++;
        }
    }
    
    return false;
}

void EcCmd::feed_motors(std::vector<MR> motors_references)
{
    for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : motors_references ) {
        if(!_client_alive){
            _consoleLog->error("Client not alive, please stop the main process!");
            return;
        }
        
        std::string msg="";
        iit::advr::Motors_PDO_cmd_Moto_PDO_cmd motor_pdo_cmd;
        
        motor_pdo_cmd.set_motor_id(bId);
        motor_pdo_cmd.set_pos_ref(pos);
        motor_pdo_cmd.set_vel_ref(vel);
        motor_pdo_cmd.set_tor_ref(tor);
        
        if ( ! iit::advr::Gains_Type_IsValid(ctrl_type) ) {
            _consoleLog->error("Wrong control type");
            return;
        }
            
        auto _ctrl_type = static_cast<iit::advr::Gains_Type>(ctrl_type);
        motor_pdo_cmd.mutable_gains()->set_type(_ctrl_type);
        
        if ( (_ctrl_type == iit::advr::Gains_Type_POSITION ||
            _ctrl_type == iit::advr::Gains_Type_VELOCITY)) {
            motor_pdo_cmd.mutable_gains()->set_pos_kp(g0);
            motor_pdo_cmd.mutable_gains()->set_pos_kd(g2);
            motor_pdo_cmd.mutable_gains()->set_tor_kp(0.0);
            motor_pdo_cmd.mutable_gains()->set_tor_ki(0.0);
            motor_pdo_cmd.mutable_gains()->set_tor_kd(0.0);
        } else if ( _ctrl_type == iit::advr::Gains_Type_IMPEDANCE) {
            motor_pdo_cmd.mutable_gains()->set_pos_kp(g0);
            motor_pdo_cmd.mutable_gains()->set_pos_kd(g1);
            motor_pdo_cmd.mutable_gains()->set_tor_kp(g2);
            motor_pdo_cmd.mutable_gains()->set_tor_ki(g3);
            motor_pdo_cmd.mutable_gains()->set_tor_kd(g4);
        } else {
            _consoleLog->error("Control type not handled");
            return;
        }
        
        
        auto op_msg = static_cast<iit::advr::AuxPDO_Op>(op);
        motor_pdo_cmd.mutable_aux_pdo()->set_op(op_msg);
        motor_pdo_cmd.mutable_aux_pdo()->set_idx(idx);
        motor_pdo_cmd.mutable_aux_pdo()->set_value(aux);
        
        _ec_zmq_cmd->Motors_PDO_cmd(motor_pdo_cmd,msg);
    }
}

//******************************* COMMANDS *****************************************************//





