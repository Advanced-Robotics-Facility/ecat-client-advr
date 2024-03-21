#include "mechanism/zmq/ec_repl_cmd.h"
#include <iostream>

using namespace zmq;
using namespace iit::advr;
using namespace std;

EcReplCmd::EcReplCmd(string zmq_uri,int timeout) :
_zmq_uri(zmq_uri),_timeout(timeout)
{   
    _context = std::make_shared<context_t>(1);
    
    _publisher = std::make_shared<socket_t>(*_context, ZMQ_REQ);

   _publisher->setsockopt(ZMQ_LINGER, 0);
   _publisher->setsockopt(ZMQ_RCVTIMEO, timeout);
   _publisher->setsockopt(ZMQ_CONNECT_TIMEOUT, 1);
   _publisher->connect(zmq_uri);
};

std::string EcReplCmd::get_zmq_uri()
{
    return _zmq_uri;
}

int EcReplCmd::get_zmq_timeout()
{
    return _timeout;
}

void EcReplCmd::set_zmq_timeout(int timeout)
{
    _timeout=timeout;
}

void EcReplCmd::set_new_timeout(int timeout)
{
    _publisher->setsockopt(ZMQ_RCVTIMEO, timeout);
}

void EcReplCmd::zmq_cmd_send(std::string m_cmd,
                         iit::advr::Repl_cmd  pb_cmd)
{
    std::string pb_msg_serialized;
    zmq::multipart_t multipart;
    try{
        /***** Protocol buffer Serialization */////
        pb_cmd.SerializeToString(&pb_msg_serialized);
        /***** ZMQ MECHANISM */////
        multipart.push(message_t(pb_msg_serialized.c_str(), pb_msg_serialized.length()));
        multipart.push(message_t(m_cmd.c_str(), m_cmd.length()));
        multipart.send((*_publisher),ZMQ_NOBLOCK);
    } catch(exception e){
        
    }
}

void EcReplCmd::zmq_cmd_recv_no_block(EcReplFault &fault)
{
    message_t update;
    fault.set_type(EC_REPL_CMD_STATUS::OK);
    fault.set_info("No fault: Good communication");
    fault.set_recovery_info("None");
    
    try{
        if(_publisher->recv(&update,ZMQ_NOBLOCK)){
            
        }
    }
    catch(exception e){
        
    }
}

void EcReplCmd::zmq_cmd_recv(string& msg,
                             iit::advr::CmdType cmd_sent,
                             EcReplFault &fault)
{
    msg.clear();
    
    message_t update;
    iit::advr::Cmd_reply pb_reply;
    try{
        if(_publisher->recv(&update))
        {
            pb_reply.ParseFromArray(update.data(),update.size());

            msg=pb_reply.msg();
            
            if(pb_reply.type()==Cmd_reply::ACK)
            {
                if(pb_reply.cmd_type()==cmd_sent)
                {
                    if(pb_reply.msg()!="")
                    {
                        fault.set_type(EC_REPL_CMD_STATUS::OK);
                        fault.set_info("No fault: Good communication");
                        fault.set_recovery_info("None");
                        return;
                    }
                    else
                    {
                        fault.set_type(EC_REPL_CMD_STATUS::WRONG_FB_MSG);
                        fault.set_info("Bad communication: Wrong message");
                        fault.set_recovery_info("Retry command");
                    }
                    
                }
                else
                {
                    fault.set_type(EC_REPL_CMD_STATUS::WRONG_CMD_TYPE);
                    fault.set_info("Bad communication: Received a wrong command type");
                    fault.set_recovery_info("Retry command");
                }
            }
            else
            {
                fault.set_type(EC_REPL_CMD_STATUS::NACK);
                fault.set_info("NACK: Bad request");
                fault.set_recovery_info("Retry to configure the request");
            }
        }
        else
        {
            fault.set_type(EC_REPL_CMD_STATUS::TIMEOUT);
            fault.set_info("Timeout reached, etherCAT master server might not be alive");
            fault.set_recovery_info("Restart the master or verify its status");
        }
    }
    catch(exception e){
        
    }
}

EcReplFault EcReplCmd::Ecat_Master_cmd(Ecat_Master_cmd_Type type,
                                       std::map<std::string ,std::string> args,
                                       std::string &msg)
{

    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
         
     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::ECAT_MASTER_CMD);
    pb_cmd.mutable_ecat_master_cmd()->set_type(type);    // REQUIRED VALUE
    
    set_new_timeout(_timeout);
    if(type!= Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR)
    {
        set_new_timeout(60000);// 1 minutes
        if(type==Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_START_MASTER)
        {
            /***** Key VALUE MECHANISM */////
            set_new_timeout(_timeout);
            for (std::map<std::string,std::string>::iterator it=args.begin(); it!=args.end(); ++it)   // OPTIONAL VALUE
            {
                KeyValStr *args=pb_cmd.mutable_ecat_master_cmd()->add_args();
                args->set_name(it->first);
                args->set_value(it->second);
            }
        }
    }
    
     
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("MASTER_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::ECAT_MASTER_CMD,fault);
    
    return fault;

}

EcReplFault EcReplCmd::FOE_Master(std::string filename,
                                  unsigned long int password,
                                  std::string mcu_type,
                                  long int slave_pos,
                                  long int board_id,
                                  std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
    
    if((filename=="")||(password==0))
    {
        fault.set_zmq_cmd(get_cmd_type(CmdType::FOE_MASTER));
        fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
        fault.set_info("Filename or password not set!");
        fault.set_recovery_info("Retry command");
        return fault;
    }
    
    /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::FOE_MASTER);
    
    if(slave_pos>0)
    {
        pb_cmd.mutable_foe_master()->set_slave_pos(slave_pos);  // SLAVE POSITION: OPTIONAL VALUE
    }
    else
    {
        pb_cmd.mutable_foe_master()->set_board_id(board_id);  // Default BOARD ID: OPTIONAL VALUE
    }
        
    pb_cmd.mutable_foe_master()->set_allocated_filename(&filename);  // REQUIRED VALUE
    pb_cmd.mutable_foe_master()->set_password(password);
    
    if(mcu_type!="")
    {
        pb_cmd.mutable_foe_master()->set_allocated_mcu_type(&mcu_type); // OPTIONAL VALUE
    }
    
    
    set_new_timeout(5*60000);// 5 minutes
    
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("MASTER_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::FOE_MASTER,fault);
    
    return fault;
}


EcReplFault EcReplCmd::Slave_SDO_info(Slave_SDO_info_Type type,
                                      long int board_id,
                                      std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
    
     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::SLAVE_SDO_INFO);
    pb_cmd.mutable_slave_sdo_info()->set_type(type);     // REQUIRED VALUE
    pb_cmd.mutable_slave_sdo_info()->set_board_id(board_id); // REQUIRED VALUE
    
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::SLAVE_SDO_INFO,fault);
    
    return fault;
}

EcReplFault EcReplCmd::Slave_SDO_cmd(long int board_id,
                                     std::vector<std::string> rd_sdo,
                                     std::map<std::string ,std::string> wr_sdo,
                                     std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
    
    if((!rd_sdo.empty()) && (!wr_sdo.empty()))
    {
         /***** RUTURN IF READ and WRITE SDO ARE SET  */////
        fault.set_zmq_cmd(get_cmd_type(CmdType::SLAVE_SDO_CMD));
        fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
        fault.set_info("Both rd_sdo and wr_sdo requested!");
        fault.set_recovery_info("Retry command");
        return fault;
    }
    else
    {
        /***** set protocol buffer command */////
        pb_cmd.set_type(CmdType::SLAVE_SDO_CMD);
        pb_cmd.mutable_slave_sdo_cmd()->set_board_id(board_id);  // REQUIRED VALUE
        
        /***** Read SDO */////
       if(!rd_sdo.empty())
       {
           for(int i=0; i<rd_sdo.size();i++)
           {
                pb_cmd.mutable_slave_sdo_cmd()->add_rd_sdo(rd_sdo.at(i));   // REQUIRED VALUE IF NOT WD  
           }
       }
       else if(!wr_sdo.empty())
       {
            /***** Write SDO */////
            for (std::map<std::string,std::string>::iterator it=wr_sdo.begin(); it!=wr_sdo.end(); ++it)
            {
                KeyValStr *wr_sdo=pb_cmd.mutable_slave_sdo_cmd()->add_wr_sdo(); // REQUIRED VALUE IF NOT RD 
                wr_sdo->set_name(it->first);
                wr_sdo->set_value(it->second);
            }
       }
       else
       {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            fault.set_zmq_cmd(get_cmd_type(CmdType::SLAVE_SDO_CMD));
            fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
            fault.set_info("SDO read/write empty requested!");
            fault.set_recovery_info("Retry command");
            return fault;
       }
    }
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::SLAVE_SDO_CMD,fault);
    
    return fault;
    
}

EcReplFault EcReplCmd::Flash_cmd(Flash_cmd_Type type,
                                 long int board_id,
                                 std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;

     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::FLASH_CMD);
    pb_cmd.mutable_flash_cmd()->set_type(type);    //REQUIRED VALUE 
    pb_cmd.mutable_flash_cmd()->set_board_id(board_id); //REQUIRED VALUE 
    
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::FLASH_CMD,fault);
    
    return fault;
}

void EcReplCmd::check_hhcm_motor_gains(iit::advr::Gains_Type ctrl_type,std::vector<float> &gains)
{
    if((ctrl_type == iit::advr::Gains_Type_POSITION ||
        ctrl_type == iit::advr::Gains_Type_VELOCITY)) {
        auto copy_gains=gains;
        gains[0]=copy_gains[0];
        gains[1]=copy_gains[2];
        gains[2]=0.0;
        gains[3]=0.0;
        gains[4]=copy_gains[1];
    }
}

EcReplFault EcReplCmd::Ctrl_cmd(Ctrl_cmd_Type type,
                                long int board_id,
                                float value,
                                std::vector<float> gains,
                                std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;

     
    /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::CTRL_CMD);
    pb_cmd.mutable_ctrl_cmd()->set_board_id(board_id);  //REQUIRED VALUE 
    pb_cmd.mutable_ctrl_cmd()->set_type(type);          //REQUIRED VALUE 
    
    if((type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_TEST_DONE)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_TEST_ERROR)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_DAC_TUNE)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_RUN_TORQUE_CALIB)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS))
    {
        pb_cmd.mutable_ctrl_cmd()->set_value(value);        //OPTIONAL VALUE
    }

    if(!gains.empty())   //OPTIONAL VALUE
    {
        if ( ! iit::advr::Gains_Type_IsValid(value) ) {
            fault.set_zmq_cmd(get_cmd_type(CmdType::CTRL_CMD));
            fault.set_type(EC_REPL_CMD_STATUS::WRONG_CMD_TYPE);
            fault.set_info("Bad command: Wrong control type detected");
            fault.set_recovery_info("Retry command");
            return fault;
        }
        
        Gains *gains_send = new Gains();
        auto ctrl_type_cast = static_cast<iit::advr::Gains_Type>(value);
        
        gains_send->set_type(ctrl_type_cast);
        if(_hhcm_motor_map.count(board_id)>0){
           check_hhcm_motor_gains(ctrl_type_cast,gains);
        }
        gains_send->set_pos_kp(gains[0]);
        gains_send->set_pos_kd(gains[1]);
        gains_send->set_tor_kp(gains[2]);
        gains_send->set_tor_kd(gains[3]);
        gains_send->set_tor_ki(gains[4]);
        pb_cmd.mutable_ctrl_cmd()->set_allocated_gains(gains_send);
    }
    
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::CTRL_CMD,fault);
    
    return fault;
}


EcReplFault EcReplCmd::Trajectory_Cmd(Trajectory_cmd_Type type,
                                      std::string name,
                                      long int board_id,
                                      homing_par_t homing_par,
                                      period_par_t period_par,
                                      smooth_par_t smooth_par,
                                      std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;

     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::TRJ_CMD); 
    pb_cmd.mutable_trajectory_cmd()->set_type(type);          //REQUIRED VALUE
    pb_cmd.mutable_trajectory_cmd()->set_name(name);          //REQUIRED VALUE 
    pb_cmd.mutable_trajectory_cmd()->set_board_id(board_id);  //REQUIRED VALUE 

    if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_HOMING)  //OPTIONAL VALUE
    {
        if(homing_par.x.empty())
        {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            fault.set_zmq_cmd(get_cmd_type(CmdType::TRJ_CMD));
            fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
            fault.set_info("Empy x vector for Homing Trajectory");
            fault.set_recovery_info("Retry command");
            return fault;
        }
        else
        {
            Trajectory_cmd_Homing_par *homing_par_send= new Trajectory_cmd_Homing_par();
            
            for(int i=0;i<homing_par.x.size();i++)
            {
                homing_par_send->add_x(homing_par.x[i]);  
            }
            
            pb_cmd.mutable_trajectory_cmd()->set_allocated_homing_par(homing_par_send);
        }
    } 
    else if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_SINE)  //OPTIONAL VALUE
    {

        Trajectory_cmd_Period_par *period_par_send= new Trajectory_cmd_Period_par();
        
        period_par_send->set_ampl(period_par.freq);
        period_par_send->set_freq(period_par.ampl);
        period_par_send->set_teta(period_par.teta);
        period_par_send->set_secs(period_par.secs);

        
        pb_cmd.mutable_trajectory_cmd()->set_allocated_period_par(period_par_send);
    } 
    else if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_SMOOTHER)  //OPTIONAL VALUE
    {

        if((smooth_par.x.empty()) && 
           (smooth_par.y.empty()) &&
           (smooth_par.x.size()!=smooth_par.y.size())) 
        {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            fault.set_zmq_cmd(get_cmd_type(CmdType::TRJ_CMD));
            fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
            fault.set_info("Wring dimension of x or y vectors");
            fault.set_recovery_info("Retry command");
            return fault;
        }
        else
        {
            Trajectory_cmd_Smooth_par *smooth_par_send= new Trajectory_cmd_Smooth_par();
            for(int i=0;i<smooth_par.x.size();i++)
            {
                smooth_par_send->add_x(smooth_par.x[i]);
                smooth_par_send->add_y(smooth_par.y[i]); 
            }
            
            pb_cmd.mutable_trajectory_cmd()->set_allocated_smooth_par(smooth_par_send);
        }
    }
    
    set_new_timeout(_timeout);

    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::TRJ_CMD,fault);
    
    return fault;
}

EcReplFault EcReplCmd::Trj_queue_cmd(Trj_queue_cmd_Type type,
                                     std::vector<std::string> trj_names,
                                     std::string &msg)
{   
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
     
    /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::TRJ_QUEUE_CMD); 
    pb_cmd.mutable_trj_queue_cmd()->set_type(type);          //REQUIRED VALUE
    
    if(!trj_names.empty())                                    //REQUIRED VALUE
    {
        for(int i=0; i < trj_names.size() ; i++)
        {
            pb_cmd.mutable_trj_queue_cmd()->add_trj_names(trj_names.at(i));
        }
    }
    else
    {
        /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
        fault.set_zmq_cmd(get_cmd_type(CmdType::TRJ_QUEUE_CMD));
        fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
        fault.set_info("trajectory vector names empy!");
        fault.set_recovery_info("Retry command");
        return fault;
    }
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::TRJ_QUEUE_CMD,fault);
    
    return fault;
}

EcReplFault EcReplCmd::PDOs_aux_cmd(std::vector<aux_cmd_message_t> aux_cmds,
                                    std::string &msg)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
    
    if(aux_cmds.empty())
    {
        /***** RUTURN IF aux cmds is empty  */////
        fault.set_zmq_cmd(get_cmd_type(CmdType::PDO_AUX_CMD));
        fault.set_type(EC_REPL_CMD_STATUS::WRONG_COMPOSITION);
        fault.set_info("Aux commands vector is empty");
        fault.set_recovery_info("Retry command");
        return fault;
    }

     
     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::PDO_AUX_CMD);
    
    for(size_t i=0 ; i < aux_cmds.size() ; i++)
    {
        PDOs_aux_cmd_Aux_cmd *aux_cmd_send = pb_cmd.mutable_pdos_aux_cmd()->add_aux_cmds();
        
        aux_cmd_message_t aux_cmd = aux_cmds[i];
        
        aux_cmd_send->set_board_id(aux_cmd.board_id); //REQUIRED VALUE 
        aux_cmd_send->set_type(aux_cmd.type); //REQUIRED VALUE 
    }
    
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(msg,CmdType::PDO_AUX_CMD,fault);
    
    return fault;
}

EcReplFault EcReplCmd::Motors_PDO_cmd(motors_ref_map motors_references)
{
    iit::advr::Repl_cmd  pb_cmd;
    EcReplFault fault;
    fault.set_type(1);
     
     /***** set protocol buffer command */////
    pb_cmd.set_type(CmdType::MOTOR_PDO_CMD);
    
    
    for ( const auto &[bId,motor_ref] : motors_references ) {
        const auto &[ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux]= motor_ref;
        if(ctrl_type!=0x00){
            if ( ! iit::advr::Gains_Type_IsValid(ctrl_type) ) {
                fault.set_zmq_cmd(get_cmd_type(CmdType::MOTOR_PDO_CMD));
                fault.set_type(EC_REPL_CMD_STATUS::WRONG_CMD_TYPE);
                fault.set_info("Bad command: Wrong control type detected");
                fault.set_recovery_info("Retry command");
                return fault;
            }

            Motors_PDO_cmd_Moto_PDO_cmd *motor_pdo_cmd = pb_cmd.mutable_motors_pdo_cmd()->add_motors_pdo();
            
            motor_pdo_cmd->set_motor_id(bId);
            motor_pdo_cmd->set_pos_ref(pos);
            motor_pdo_cmd->set_vel_ref(vel);
            motor_pdo_cmd->set_tor_ref(tor);
                
            auto ctrl_type_cast = static_cast<iit::advr::Gains_Type>(ctrl_type);
            motor_pdo_cmd->mutable_gains()->set_type(ctrl_type_cast);
            
            std::vector<float> gains_check={g0,g1,g2,g3,g4};
            if(_hhcm_motor_map.count(bId)>0){
                check_hhcm_motor_gains(ctrl_type_cast,gains_check);
            }
                
            motor_pdo_cmd->mutable_gains()->set_pos_kp(gains_check[0]);
            motor_pdo_cmd->mutable_gains()->set_pos_kd(gains_check[1]);
            motor_pdo_cmd->mutable_gains()->set_tor_kp(gains_check[2]);
            motor_pdo_cmd->mutable_gains()->set_tor_ki(gains_check[3]);
            motor_pdo_cmd->mutable_gains()->set_tor_kd(gains_check[4]);
            
            auto op_msg = static_cast<iit::advr::AuxPDO_Op>(op);
            motor_pdo_cmd->mutable_aux_pdo()->set_op(op_msg);
            motor_pdo_cmd->mutable_aux_pdo()->set_idx(idx);
            motor_pdo_cmd->mutable_aux_pdo()->set_value(aux);
        }
    }
    
 
    set_new_timeout(_timeout);
    /***** ZMQ Send and protocol buffer Serialization */////
    zmq_cmd_send("ESC_CMD",pb_cmd);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv_no_block(fault);
   
    return fault;
}

void EcReplCmd::set_motor_type_map(std::map<int32_t,std::string> motor_type_map)
{
    for ( const auto &[motor_id,motor_type] : motor_type_map ) {
        if(motor_type=="HHCM_MOTOR"){
            _hhcm_motor_map[motor_id]=motor_type;
        }
        else if(motor_type=="CIRCULO9_MOTOR"){
            _circulo9_motor_map[motor_id]=motor_type;
        }
        else if(motor_type=="AMC_FLEXPRO_MOTOR"){
            _amc_flexpro_motor_map[motor_id]=motor_type;
        }
    }
}


EcReplCmd::~EcReplCmd()
{
};

