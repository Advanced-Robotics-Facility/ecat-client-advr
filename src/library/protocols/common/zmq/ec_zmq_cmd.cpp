#include "ec_zmq_cmd.h"
#include <iostream>

using namespace zmq;
using namespace iit::advr;
using namespace std;

EcZmqCmd::EcZmqCmd(string zmq_uri,int timeout) :
_zmq_uri(zmq_uri),_timeout(timeout)
{   

    _context = std::make_shared<context_t>(1);
    
    _publisher = std::make_shared<socket_t>(*_context, ZMQ_REQ);

   _publisher->setsockopt(ZMQ_LINGER, 0);
   _publisher->setsockopt(ZMQ_RCVTIMEO, _timeout);
   _publisher->setsockopt(ZMQ_CONNECT_TIMEOUT, 1);
   _publisher->connect(_zmq_uri);

};

std::string EcZmqCmd::get_zmq_uri()
{
    return _zmq_uri;
}

int EcZmqCmd::get_zmq_timeout()
{
    return _timeout;
}

void EcZmqCmd::set_zmq_timeout(int timeout)
{
    _timeout=timeout;
    _publisher->setsockopt(ZMQ_RCVTIMEO, _timeout);
    
}


void EcZmqCmd::clear_zmq_client_message()
{
   /***** CLEAR protocol buffer command and reply */////
    _pb_cmd.Clear();
    _pb_reply.Clear();
    
    /***** Clear Multipart Message for communication*/////
    _multipart.clear();
    
    // Clear Serialized message
    _pb_msg_serialized="";
    
    /***** Clear feedback message*/////
    _fd_msg="";
    
    /***** Clear FAULT STRUCTURE */////
    _fault.set_zmq_cmd("");
    _fault.set_type(0);
    _fault.set_info("");
    _fault.set_recovery_info(""); 
    
}

EcZmqFault EcZmqCmd::get_fault()
{
 return _fault;
};


void EcZmqCmd::Ecat_Master_cmd(Ecat_Master_cmd_Type type,
                                    std::map<std::string ,std::string> args,
                                    std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Ecat_Master_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="MASTER_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::ECAT_MASTER_CMD);
    _pb_cmd.mutable_ecat_master_cmd()->set_type(type);    // REQUIRED VALUE
    
    if(type!= Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR)
    {
        _publisher->setsockopt(ZMQ_RCVTIMEO, 60000); // 1 minutes

        if(type==Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_START_MASTER)
        {
            /***** Key VALUE MECHANISM */////
            
            for (std::map<std::string,std::string>::iterator it=args.begin(); it!=args.end(); ++it)   // OPTIONAL VALUE
            {
                KeyValStr *args=_pb_cmd.mutable_ecat_master_cmd()->add_args();
                args->set_name(it->first);
                args->set_value(it->second);
            }
        }
    }
     
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::ECAT_MASTER_CMD);
     
     msg=_fd_msg;
}

void EcZmqCmd::FOE_Master(std::string filename,
                    unsigned long int password,
                    std::string mcu_type,
                    long int slave_pos,
                    long int board_id,
                    std::string &msg)
{
    _publisher->setsockopt(ZMQ_RCVTIMEO, 5*60000); // 5 minutes
    
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("FOE_Master");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="MASTER_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::FOE_MASTER);
    
    if((filename=="")||(password==0))
    {
        _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
        _fault.set_info("Filename or password not set!");
        _fault.set_recovery_info("Retry command");
        return;
    }
    
    if(slave_pos>0)
    {
        _pb_cmd.mutable_foe_master()->set_slave_pos(slave_pos);  // SLAVE POSITION: OPTIONAL VALUE
    }
    else
    {
        _pb_cmd.mutable_foe_master()->set_board_id(board_id);  // Default BOARD ID: OPTIONAL VALUE
    }
        
    _pb_cmd.mutable_foe_master()->set_allocated_filename(&filename);  // REQUIRED VALUE
    _pb_cmd.mutable_foe_master()->set_password(password);
    
    if(mcu_type!="")
    {
        _pb_cmd.mutable_foe_master()->set_allocated_mcu_type(&mcu_type); // OPTIONAL VALUE
    }
    
    /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);
    
    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(_fd_msg,CmdType::FOE_MASTER);
    
    msg=_fd_msg;
}


void EcZmqCmd::Slave_SDO_info(Slave_SDO_info_Type type,
                                   long int board_id,
                                   std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Slave_SDO_info");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::SLAVE_SDO_INFO);
    _pb_cmd.mutable_slave_sdo_info()->set_type(type);     // REQUIRED VALUE
    _pb_cmd.mutable_slave_sdo_info()->set_board_id(board_id); // REQUIRED VALUE
    
    
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::SLAVE_SDO_INFO);
     
     msg=_fd_msg;
}

void EcZmqCmd::Slave_SDO_cmd(long int board_id,
                                  std::vector<std::string> rd_sdo,
                                  std::map<std::string ,std::string> wr_sdo,
                                  std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Slave_SDO_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::SLAVE_SDO_CMD);
    _pb_cmd.mutable_slave_sdo_cmd()->set_board_id(board_id);  // REQUIRED VALUE
    
    if((!rd_sdo.empty()) && (!wr_sdo.empty()))
    {
         /***** RUTURN IF READ and WRITE SDO ARE SET  */////
        _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
        _fault.set_info("Both rd_sdo and wr_sdo requested!");
        _fault.set_recovery_info("Retry command");
        return;
    }
    else
    {
        /***** Read SDO */////
       if(!rd_sdo.empty())
       {
           for(int i=0; i<rd_sdo.size();i++)
           {
           _pb_cmd.mutable_slave_sdo_cmd()->add_rd_sdo(rd_sdo.at(i));   // REQUIRED VALUE IF NOT WD  
           }
       }
       else if(!wr_sdo.empty())
       {
            /***** Write SDO */////
            for (std::map<std::string,std::string>::iterator it=wr_sdo.begin(); it!=wr_sdo.end(); ++it)
            {
                KeyValStr *wr_sdo=_pb_cmd.mutable_slave_sdo_cmd()->add_wr_sdo(); // REQUIRED VALUE IF NOT RD 
                wr_sdo->set_name(it->first);
                wr_sdo->set_value(it->second);
            }
       }
       else
       {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
            _fault.set_info("SDO read/write empty requested!");
            _fault.set_recovery_info("Retry command");
            return;
       }
    }
    
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);
    
    zmq_cmd_recv(_fd_msg,CmdType::SLAVE_SDO_CMD);
    
    msg=_fd_msg;
    
}

void EcZmqCmd::Flash_cmd(Flash_cmd_Type type,
                              long int board_id,
                              std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Flash_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::FLASH_CMD);
    _pb_cmd.mutable_flash_cmd()->set_type(type);    //REQUIRED VALUE 
    _pb_cmd.mutable_flash_cmd()->set_board_id(board_id); //REQUIRED VALUE 
   
    
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::FLASH_CMD);
     
     msg=_fd_msg;
}

void EcZmqCmd::Ctrl_cmd(Ctrl_cmd_Type type,
                             long int board_id,
                             float value,
                             std::vector<float> gains,
                             std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Ctrl_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";    
     
      /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::CTRL_CMD);
    _pb_cmd.mutable_ctrl_cmd()->set_board_id(board_id);  //REQUIRED VALUE 
    _pb_cmd.mutable_ctrl_cmd()->set_type(type);          //REQUIRED VALUE 
    
    if((type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_TEST_DONE)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_TEST_ERROR)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_DAC_TUNE)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_RUN_TORQUE_CALIB)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP)&&
       (type!=Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS))
    {
        _pb_cmd.mutable_ctrl_cmd()->set_value(value);        //OPTIONAL VALUE
    }

    if(!gains.empty())   //OPTIONAL VALUE
    {
        Gains *gains_send = new Gains();
        if((value == 0x3B ) || (value == 0x71 ))
        {
         
            /* SET CONTROL MODE*///
            if(value == 0x3B ) 
            {
                gains_send->set_type(Gains_Type::Gains_Type_POSITION);
            }
            else
            {
                gains_send->set_type(Gains_Type::Gains_Type_VELOCITY);
            }
            
            /* SET GAINS *///
            gains_send->set_pos_kp(gains[0]);
            gains_send->set_pos_kd(gains[2]);
            gains_send->set_tor_kp(0.0);
            gains_send->set_tor_kd(0.0);
            gains_send->set_tor_ki(gains[1]);    
            
        }
        else
        {
            /* SET CONTROL MODE AND GAINS *///
            gains_send->set_type(Gains_Type::Gains_Type_IMPEDANCE);
            gains_send->set_pos_kp(gains[0]);
            gains_send->set_pos_kd(gains[1]);
            gains_send->set_tor_kp(gains[2]);
            gains_send->set_tor_kd(gains[3]);
            gains_send->set_tor_ki(gains[4]);  // this is fc!
        }
        
        _pb_cmd.mutable_ctrl_cmd()->set_allocated_gains(gains_send);
    }
    
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::CTRL_CMD);
     
     msg=_fd_msg;
    
    
}


void EcZmqCmd::Trajectory_Cmd(Trajectory_cmd_Type type,
                                   std::string name,
                                   long int board_id,
                                   homing_par_t homing_par,
                                   period_par_t period_par,
                                   smooth_par_t smooth_par,
                                   std::string &msg)
{
    
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Trajectory_Cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";    
     
      /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::TRJ_CMD); 
    _pb_cmd.mutable_trajectory_cmd()->set_type(type);          //REQUIRED VALUE
    _pb_cmd.mutable_trajectory_cmd()->set_name(name);          //REQUIRED VALUE 
    _pb_cmd.mutable_trajectory_cmd()->set_board_id(board_id);  //REQUIRED VALUE 
    
    if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_HOMING)  //OPTIONAL VALUE
    {
        if(homing_par.x.empty())
        {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
            _fault.set_info("Empy x vector for Homing Trajectory");
            _fault.set_recovery_info("Retry command");
            return;
        }
        else
        {
            Trajectory_cmd_Homing_par *homing_par_send= new Trajectory_cmd_Homing_par();
            
            for(int i=0;i<homing_par.x.size();i++)
            {
            homing_par_send->add_x(homing_par.x[i]);  
            }
            
            _pb_cmd.mutable_trajectory_cmd()->set_allocated_homing_par(homing_par_send);
        }
    } 
    else if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_SINE)  //OPTIONAL VALUE
    {

        Trajectory_cmd_Period_par *period_par_send= new Trajectory_cmd_Period_par();
        
        period_par_send->set_ampl(period_par.freq);
        period_par_send->set_freq(period_par.ampl);
        period_par_send->set_teta(period_par.teta);
        period_par_send->set_secs(period_par.secs);

        
        _pb_cmd.mutable_trajectory_cmd()->set_allocated_period_par(period_par_send);
    } 
    else if(type==Trajectory_cmd_Type::Trajectory_cmd_Type_SMOOTHER)  //OPTIONAL VALUE
    {

        if((smooth_par.x.empty()) && 
           (smooth_par.y.empty()) &&
           (smooth_par.x.size()!=smooth_par.y.size())) 
        {
            /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
            _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
            _fault.set_info("Wring dimension of x or y vectors");
            _fault.set_recovery_info("Retry command");
            return;
        }
        else
        {
            Trajectory_cmd_Smooth_par *smooth_par_send= new Trajectory_cmd_Smooth_par();
            for(int i=0;i<smooth_par.x.size();i++)
            {
                smooth_par_send->add_x(smooth_par.x[i]);
                smooth_par_send->add_y(smooth_par.y[i]); 
            }
            
            _pb_cmd.mutable_trajectory_cmd()->set_allocated_smooth_par(smooth_par_send);
        }
    }
    
      /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::TRJ_CMD);
     
     msg=_fd_msg;
     
}

void EcZmqCmd::Trj_queue_cmd(Trj_queue_cmd_Type type,
                                  std::vector<std::string> trj_names,
                                  std::string &msg)
{   
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Trj_queue_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";    
     
      /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::TRJ_QUEUE_CMD); 
    _pb_cmd.mutable_trj_queue_cmd()->set_type(type);          //REQUIRED VALUE
    
    if(!trj_names.empty())                                    //REQUIRED VALUE
    {
        for(int i=0; i < trj_names.size() ; i++)
        {
            _pb_cmd.mutable_trj_queue_cmd()->add_trj_names(trj_names.at(i));
        }
    }
    else
    {
        /***** RUTURN IF BOTH READ and WRITE SDO ARE NOT SET  */////
        _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
        _fault.set_info("trajectory vector names empy!");
        _fault.set_recovery_info("Retry command");
        return;
    }
    
       /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::TRJ_QUEUE_CMD);
     
     msg=_fd_msg;

}

void EcZmqCmd::PDOs_aux_cmd(std::vector<aux_cmd_message_t> aux_cmds,
                                 std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();
    
    if(aux_cmds.empty())
    {
        /***** RUTURN IF aux cmds is empty  */////
        _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_COMPOSITION);
        _fault.set_info("Aux commands vector is empty");
        _fault.set_recovery_info("Retry command");
        return;
    }

    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("PDOs_aux_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::PDO_AUX_CMD);
    
    for(size_t i=0 ; i < aux_cmds.size() ; i++)
    {
        PDOs_aux_cmd_Aux_cmd *aux_cmd_send = _pb_cmd.mutable_pdos_aux_cmd()->add_aux_cmds();
        
        aux_cmd_message_t aux_cmd = aux_cmds[i];
        
        aux_cmd_send->set_board_id(aux_cmd.board_id); //REQUIRED VALUE 
        aux_cmd_send->set_type(aux_cmd.type); //REQUIRED VALUE 
    }
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
     zmq_cmd_recv(_fd_msg,CmdType::PDO_AUX_CMD);
     
     msg=_fd_msg;
}

void EcZmqCmd::Motors_PDO_cmd(motors_ref_t refs,
                              std::string &msg)
{
    // CLEAR ALL ZMQ STRUCTURES
    clear_zmq_client_message();
    
    /***** Clear feedback message*/////
    msg="";
    
    // Set ZMQ CMD */////
    _fault.set_zmq_cmd("Motors_PDO_cmd");
    
    /***** MESSAGE HEADER*/////
     _m_cmd="ESC_CMD";
     
     /***** set protocol buffer command */////
    _pb_cmd.set_type(CmdType::MOTOR_PDO_CMD);
    
    
    for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : refs ) {

        Motors_PDO_cmd_Moto_PDO_cmd *motor_pdo_cmd = _pb_cmd.mutable_motors_pdo_cmd()->add_motors_pdo();
        
        motor_pdo_cmd->set_motor_id(bId);
        motor_pdo_cmd->set_pos_ref(pos);
        motor_pdo_cmd->set_vel_ref(vel);
        motor_pdo_cmd->set_tor_ref(tor);
        
        if ( ! iit::advr::Gains_Type_IsValid(ctrl_type) ) {
            _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_CMD_TYPE);
            _fault.set_info("Bad command: Wrong control type detected");
            _fault.set_recovery_info("Retry command");
            return;
        }
            
        auto _ctrl_type = static_cast<iit::advr::Gains_Type>(ctrl_type);
        motor_pdo_cmd->mutable_gains()->set_type(_ctrl_type);
        
        if ( (_ctrl_type == iit::advr::Gains_Type_POSITION ||
            _ctrl_type == iit::advr::Gains_Type_VELOCITY)) {
            motor_pdo_cmd->mutable_gains()->set_pos_kp(g0);
            motor_pdo_cmd->mutable_gains()->set_pos_kd(g2);
            motor_pdo_cmd->mutable_gains()->set_tor_kp(0.0);
            motor_pdo_cmd->mutable_gains()->set_tor_ki(0.0);
            motor_pdo_cmd->mutable_gains()->set_tor_kd(0.0);
        } else if ( _ctrl_type == iit::advr::Gains_Type_IMPEDANCE) {
            motor_pdo_cmd->mutable_gains()->set_pos_kp(g0);
            motor_pdo_cmd->mutable_gains()->set_pos_kd(g1);
            motor_pdo_cmd->mutable_gains()->set_tor_kp(g2);
            motor_pdo_cmd->mutable_gains()->set_tor_ki(g3);
            motor_pdo_cmd->mutable_gains()->set_tor_kd(g4);
        } else {
            _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_CMD_TYPE);
            _fault.set_info("Bad command: Control type not handled");
            _fault.set_recovery_info("Retry command");
            return;
        }
        
        
        auto op_msg = static_cast<iit::advr::AuxPDO_Op>(op);
        motor_pdo_cmd->mutable_aux_pdo()->set_op(op_msg);
        motor_pdo_cmd->mutable_aux_pdo()->set_idx(idx);
        motor_pdo_cmd->mutable_aux_pdo()->set_value(aux);
    }
    
 
     /***** Protocol buffer Serialization */////
    _pb_cmd.SerializeToString(&_pb_msg_serialized);
    /***** ZMQ MECHANISM */////
    _multipart.push(message_t(_pb_msg_serialized.c_str(), _pb_msg_serialized.length()));
    _multipart.push(message_t(_m_cmd.c_str(), _m_cmd.length()));
    _multipart.send((*_publisher),ZMQ_NOBLOCK);

    /***** ZMQ Received and protocol buffer De-Serialization */////
    zmq_cmd_recv(_fd_msg,CmdType::MOTOR_PDO_CMD);
     
     msg=_fd_msg;
    
}

void EcZmqCmd::zmq_cmd_recv(string& msg,CmdType cmd_sent)
{
    message_t update;

    if(_publisher->recv(&update))
    {
        _pb_reply.ParseFromArray(update.data(),update.size());

        msg=_pb_reply.msg();
        
        if(_pb_reply.type()==Cmd_reply::ACK)
        {
            if(_pb_reply.cmd_type()==cmd_sent)
            {
                if(_pb_reply.msg()!="")
                {
                    _fault.set_type(EC_ZMQ_CMD_STATUS::OK);
                    _fault.set_info("No fault: Good communication");
                    _fault.set_recovery_info("None");
                    return;
                }
                else
                {
                    _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_FB_MSG);
                    _fault.set_info("Bad communication: Wrong message");
                    _fault.set_recovery_info("Retry command");
                }
                
            }
            else
            {
                _fault.set_type(EC_ZMQ_CMD_STATUS::WRONG_CMD_TYPE);
                _fault.set_info("Bad communication: Received a wrong command type");
                _fault.set_recovery_info("Retry command");
            }
        }
        else
        {
            _fault.set_type(EC_ZMQ_CMD_STATUS::NACK);
            _fault.set_info("NACK: Bad request");
            _fault.set_recovery_info("Retry to configure the request");
        }
    }
    else
    {
        _fault.set_type(EC_ZMQ_CMD_STATUS::TIMEOUT);
        _fault.set_info("Timeout reached, etherCAT master server might not be alive");
        _fault.set_recovery_info("Restart the master or verify its status");
        _publisher->close();
    }
    
}

EcZmqCmd::~EcZmqCmd()
{
};

