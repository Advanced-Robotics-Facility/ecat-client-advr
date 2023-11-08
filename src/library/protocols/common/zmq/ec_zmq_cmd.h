#ifndef __EC_ZMQ_CMD__
#define __EC_ZMQ_CMD__

#include <protobuf/repl_cmd.pb.h>
#include <protobuf/ecat_pdo.pb.h>

#include "zmq.hpp"
#include <zmq_addon.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

enum EC_ZMQ_CMD_STATUS : int {

    TIMEOUT=            0,
    NACK=               1,
    WRONG_COMPOSITION=  2,
    WRONG_CMD_TYPE=     3,
    WRONG_FB_MSG=       4, //FB=feedback
    OK=                 5
    
};

class EcZmqFault
{
public:
    
    /**
    * @brief Constructor of EcZmqCmd Fault Class.
    *  This class is responsible to manage the fault coming from ZMQ communication.
    *
    */
    EcZmqFault(){
         
    };
    
    /**
    * @brief Destructor of EcZmqCmd Fault Class.
    * 
    */
    ~EcZmqFault(){
        
    };
    
    /**
    * @brief method to set the ZMQ Command type.
    * 
    * @param zmq_cmd p_zmq_cmd:...
    */
    void set_zmq_cmd(std::string zmq_cmd){
    _zmq_cmd=zmq_cmd;
    };
    
    
    /**
    * @brief method to set the type of ZMQ Command fault.
    * 
    * @param type p_type:...
    */
    void set_type(int type){
    _type=type;
    };
    
    
    /**
    * @brief method to set the information related on ZMQ Command fault.
    * 
    * @param info p_info:...
    */
    void set_info(std::string info){
    _info=info;
    };
    
    
    /**
    * @brief method to se the recovery for the ZMQ Command.
    * 
    * @param recovery_info p_recovery_info:...
    */
    void set_recovery_info(std::string recovery_info){
    _recovery_info=recovery_info;
    };
    
    
    /**
    * @brief method to get the ZMQ Command type
    * 
    * @return std::__cxx11::string
    */
    std::string get_zmq_cmd(){
    return _zmq_cmd;
    };
    
    
    /**
    * @brief method to get the type of ZMQ Command fault.
    * 
    * @return int
    */
    int get_type(){
    return _type;
    };
    
    
    /**
    * @brief method to get the recovery for the ZMQ Command.
    * 
    * @return std::__cxx11::string
    */
    std::string get_info(){
    return _info;
    };
    
    
    /**
    * @brief method to set the information related on ZMQ Command fault.
    * 
    * @return std::__cxx11::string
    */
    std::string get_recovery_info(){
    return _recovery_info;
    };
    
private:
    int _type;
    std::string _info;
    std::string _recovery_info;
    std::string _zmq_cmd;
};

class EcZmqCmd
{
public:
    
    typedef std::shared_ptr<EcZmqCmd> Ptr;
    
    struct homing_par_t{
    
        std::vector<float> x;

    };
    
    struct period_par_t{
    
        float freq;
        float ampl;
        float teta;
        float secs;
        
    };
    
    struct smooth_par_t{
    
        std::vector<float> x;
        std::vector<float> y;
        
    };

    struct aux_cmd_message_t{
    
        iit::advr::PDOs_aux_cmd_Aux_cmd_Type type;
        long int board_id;
        
    };
    
    
    /**
    * @brief Constructor of EcZmqCmd Class.
    * It needs of zmq uri (TCP/UDP) and of a timeout.
    * 
    * @param zmq_uri p_zmq_uri: ZMQ URI to create the communication.
    * @param timeout p_timeout: Timeout for the communication.
    */
    EcZmqCmd(std::string zmq_uri,int timeout);
    
    /**
    * @brief Destructor of EcZmqCmd Class-
    * 
    */
    ~EcZmqCmd();
    
    
    /**
    * @brief EtherCAT Master command for stopping, starting the EtherCAT Master Server 
    *        and GET EtherCAT information of the slaves.
    * 
    * @param type Ecat_Master_cmd_Type: START_MASTER, STOP_MASTER, GET_SLAVES_DESCR.
    * @param args std::map of string :  for istance start the master with some parameters... ({‘app_mode’:’run_mode’,’use_ecat_pos_as_id’:’false’}). 
    * @param msg  string msg: return feedback message from ZMQ communication.
    */
    void Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type type,
                         std::map<std::string,std::string> args,
                         std::string &msg);
    
   
    /**
    * @brief FOE Master command for flash a new firmare for a specific slave.
    * 
    * @param filename p_filename: Filename to use to flah.
    * @param password p_password: Password for flashing  (0xDAD0 [c28] 0xA550 [m3])
    * @param mcu_type p_mcu_type: Microcontroller type m3 or c28.
    * @param slave_pos p_slave_pos: Slave position on the EtherCAT Network.
    * @param board_id p_board_id: Board Id of the slave.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void FOE_Master(std::string filename,
                    unsigned long int password,
                    std::string mcu_type,
                    long int slave_pos,
                    long int board_id,
                    std::string &msg);
    
    /**
    * @brief Slave SDO Information is a command to get the SDO datas of a specific slave.
    * 
    * @param type p_type: It's possible to get the SDO information with the NAME or OBJECT DICTIONARY.
    * @param board_id p_board_id: Board Id of the slave.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Slave_SDO_info(iit::advr::Slave_SDO_info_Type type,
                        long int board_id,
                        std::string &msg);
    
    /**
    * @brief The slave SDO command is typical used to read the values of SDO of specific slave 
    * in pre-operational and operational state of the EtherCAT Master Server.
    * This command is also used to write the values of the SDO in calibration phase ("pre-operational"). 
    * 
    * @param board_id p_board_id: Board Id of the slave.
    * @param rd_sdo p_rd_sdo: read sdo vector to read the values specified in this structure.
    * @param wr_sdo p_wr_sdo: write sdo map to write, using the mechanism of Key-Value (map of SDO type-value),
    *                         the values of the SDO specified in this structure. 
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Slave_SDO_cmd(long int board_id,
                       std::vector<std::string> rd_sdo,
                       std::map<std::string ,std::string> wr_sdo,
                       std::string &msg);
    
    /**
    * @brief Flash command for save, load and load default sdo of the FLASH
    * 
    * @param type p_type: Save, Load and Load default SDO parameters of the flash. 
    * @param board_id p_board_id: Board Id of the slave.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Flash_cmd(iit::advr::Flash_cmd_Type type,
                   long int board_id,
                   std::string &msg);
    
    
    /**
    * @brief Control command is used to control the motors or start the auto-calibration mechanisms.
    * 
    * @param type p_type: Type of control command, i.e position: send a position to the slave.
    * @param board_id p_board_id: Board Id of the slave.
    * @param value p_value: value to send (rad, rad/s, Nm or ON/OFF or nothing).
    * @param gains p_gains: Optional value to set the gains of the slave.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Ctrl_cmd(iit::advr::Ctrl_cmd_Type type,
                  long int board_id,
                  float value,
                  std::vector<float> gains,
                  std::string &msg);
    
    
    /**
    * @brief Command to send a trajectory to the slave.
    * 
    * @param type p_type: Homing, Sine, Smooth Trajectory. 
    * @param name p_name: Name of trajectory.
    * @param board_id p_board_id: Board Id of the slave.
    * @param homing_par p_homing_par: Structure of homing trajectory that has the time values to perform the hominh position.
    * @param period_par p_period_par: Structure of periodic trajectory, in this case, a sine wave with frequency, amplitude, theta (offset), and time,
    * @param smooth_par p_smooth_par: Structure of smooth trajectory having like x the time and y the position in radians.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Trajectory_Cmd(iit::advr::Trajectory_cmd_Type type,
                        std::string name,
                        long int board_id,
                        homing_par_t homing_par,
                        period_par_t period_par,
                        smooth_par_t smooth_par,
                        std::string &msg);
    
    /**
    * @brief Command to start or clear a trajectoy using its name.
    * 
    * @param type p_type: Push (start) or clear the trajectory.
    * @param trj_names p_trj_names: Name of Trajectory.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Trj_queue_cmd(iit::advr::Trj_queue_cmd_Type type,
                        std::vector<std::string> trj_names,
                        std::string &msg);
    
        /**
    * @brief Command to release or engage motors brake
    * 
    * @param aux_cmds aux_cmds: vector of aux cmd setting slave id and release or engage brakes.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void PDOs_aux_cmd(std::vector<aux_cmd_message_t> aux_cmds,
                      std::string &msg);
    
    /**
    * @brief Command to send references 
    * 
    * @param aux_cmds ref: references.
    * @param msg p_msg: return feedback message from ZMQ communication.
    */
    void Motors_PDO_cmd(iit::advr::Motors_PDO_cmd_Moto_PDO_cmd ref,
                        std::string &msg);
    
    
    /**
    * @brief This method is responsible to wait the reply of ZMQ EtherCAT Server, 
    * finding communication errors and and filling the EcZmqFault class. 
    * 
    * @param msg p_msg: return feedback message from ZMQ communication.
    * @param cmd_sent p_cmd_sent: Checking of the command reply with the command sent. 
    */
    void zmq_cmd_recv(std::string& msg,
                      iit::advr::CmdType cmd_sent);
    
    /**
    * @brief Return ZMQ URI for the ZMQ communication. 
    * 
    * @return std::__cxx11::string
    */
    std::string get_zmq_uri();
    
    /**
    * @brief Return timeout of ZMQ communication.
    * 
    * @return int
    */
    int get_zmq_timeout();
    
    /**
    * @brief set timeout of ZMQ communication.
    * 
    * @param timeout p_timeout:...
    */
    void set_zmq_timeout(int timeout);
    
    /**
    * @brief Clear all structures of ZMQ communication 
    * (protocol buffer command and reply,
    *  zmq multipart message, 
    *  protocol buffer serialized,
    *  feedback message and the Fault object).
    * 
    */
    void clear_zmq_client_message();
    
    /**
    * @brief Return fault object of ZMQ communication.
    * 
    * @return EcZmqFault
    */
    EcZmqFault get_fault();

private:
    
    zmq::multipart_t _multipart;
    
    iit::advr::Repl_cmd  _pb_cmd;
    iit::advr::Cmd_reply _pb_reply;
    
    std::string _zmq_uri,_m_cmd,_pb_msg_serialized,_fd_msg;
    int _timeout;
    
    std::shared_ptr<zmq::context_t> _context;
    std::shared_ptr<zmq::socket_t>  _publisher;
    EcZmqFault _fault;
  
};

#endif

