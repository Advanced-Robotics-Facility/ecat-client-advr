#ifndef __EC_REPL_CMD__
#define __EC_REPL_CMD__

#include "mechanism/zmq/ec_zmq_context.h"

enum EC_REPL_CMD_STATUS : int {

    TIMEOUT=            0,
    NACK=               1,
    WRONG_COMPOSITION=  2,
    WRONG_CMD_TYPE=     3,
    WRONG_FB_MSG=       4, //FB=feedback
    OK=                 5
    
};

inline const char* get_cmd_type(iit::advr::CmdType type)
{
    switch (type)
    {
        case iit::advr::CmdType::TRJ_CMD: return "TRJ_CMD";
        case iit::advr::CmdType::CTRL_CMD: return "CTRL_CMD";
        case iit::advr::CmdType::FLASH_CMD: return "FLASH_CMD";
        case iit::advr::CmdType::ECAT_MASTER_CMD: return "ECAT_MASTER_CMD";
        case iit::advr::CmdType::FOE_MASTER: return "FOE_MASTER";
        case iit::advr::CmdType::TRJ_QUEUE_CMD: return "TRJ_QUEUE_CMD";
        case iit::advr::CmdType::SLAVE_SDO_CMD: return "SLAVE_SDO_CMD";
        case iit::advr::CmdType::SLAVE_SDO_INFO: return "SLAVE_SDO_INFO";
        case iit::advr::CmdType::MOTOR_PDO_CMD: return "MOTOR_PDO_CMD";
        case iit::advr::CmdType::PDO_AUX_CMD: return "PDO_AUX_CMD";
        default: return("UNKNOWN COMMAND");
    }
}


class EcReplFault
{
public:
    
    /**
    * @brief Constructor of EcReplCmd Fault Class.
    *  This class is responsible to manage the fault coming from ZMQ communication.
    *
    */
    EcReplFault(){
         
    };
    
    /**
    * @brief Destructor of EcReplCmd Fault Class.
    * 
    */
    ~EcReplFault(){
        
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


class EcReplCmd
{
public:
    
    typedef std::shared_ptr<EcReplCmd> Ptr;
    
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
    
    using m_ref_t = std::tuple<int32_t,                              //ctrl_type
                               float, float, float,                  // pos_ref, vel_ref, tor_ref
                               float, float, float, float, float,    // gains
                               uint32_t, uint32_t, float>;           // op, idx, aux

    using motors_ref_map = std::map<int32_t,m_ref_t>;
    
    
    /**
    * @brief Constructor of EcReplCmd Class.
    * It needs of zmq uri (TCP/UDP) and of a timeout.
    * 
    * @param zmq_uri p_zmq_uri: ZMQ URI to create the communication.
    * @param timeout p_timeout: Timeout for the communication.
    */
    EcReplCmd(std::string zmq_uri,int timeout);
    
    /**
    * @brief Destructor of EcReplCmd Class-
    * 
    */
    ~EcReplCmd();

    
    /**
    * @brief EtherCAT Master command for stopping, starting the EtherCAT Master Server 
    *        and GET EtherCAT information of the slaves.
    * 
    * @param type Ecat_Master_cmd_Type: START_MASTER, STOP_MASTER, GET_SLAVES_DESCR.
    * @param args std::map of string :  for istance start the master with some parameters... ({‘app_mode’:’run_mode’,’use_ecat_pos_as_id’:’false’}). 
    * @param msg  string msg: return feedback message from ZMQ communication.
    * @return fault
    */
    EcReplFault Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type type,
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
    * @return fault
    */
    EcReplFault FOE_Master(std::string filename,
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
    * @return fault
    */
    EcReplFault Slave_SDO_info(iit::advr::Slave_SDO_info_Type type,
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
    * @return fault
    */
    EcReplFault Slave_SDO_cmd(long int board_id,
                            std::vector<std::string> rd_sdo,
                            std::map<std::string ,std::string> wr_sdo,
                            std::string &msg);
    
    /**
    * @brief Flash command for save, load and load default sdo of the FLASH
    * 
    * @param type p_type: Save, Load and Load default SDO parameters of the flash. 
    * @param board_id p_board_id: Board Id of the slave.
    * @param msg p_msg: return feedback message from ZMQ communication.
    * @return fault
    */
    EcReplFault Flash_cmd(iit::advr::Flash_cmd_Type type,
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
    * @return fault
    */
    EcReplFault Ctrl_cmd(iit::advr::Ctrl_cmd_Type type,
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
    * @return fault
    */
    EcReplFault Trajectory_Cmd(iit::advr::Trajectory_cmd_Type type,
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
    * @return fault
    */
    EcReplFault Trj_queue_cmd(iit::advr::Trj_queue_cmd_Type type,
                             std::vector<std::string> trj_names,
                             std::string &msg);
    
        /**
    * @brief Command to release or engage motors brake
    * 
    * @param aux_cmds aux_cmds: vector of aux cmd setting slave id and release or engage brakes.
    * @param msg p_msg: return feedback message from ZMQ communication.
    * @return fault
    */
    EcReplFault PDOs_aux_cmd(std::vector<aux_cmd_message_t> aux_cmds,
                            std::string &msg);
    
    /**
    * @brief Command to send references 
    * 
    * @param refs : references motors map.
    * @return fault
    */
    EcReplFault Motors_PDO_cmd(motors_ref_map motors_references);
    
    
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
    
    void set_motor_type_map(std::map<int32_t,std::string> motor_type_map);

private:
    
    std::string _zmq_uri;
    int _timeout;
    std::map<int32_t,std::string> _advrf_motor_map;
    std::map<int32_t,std::string> _synapticon_motor_map;
    void check_advrf_motor_gains(iit::advr::Gains_Type ctrl_type,std::vector<float> &gains);


    /** 
    * @brief This method is responsible to perform the ZMQ commad to EtherCAT Server
    * @param pb_cmd protobuf command
    * @param msg return feedback message from ZMQ communication.
    * @param timeout timeout
    * @param fault fault detected. 
    */

    void zmq_do_cmd(iit::advr::Repl_cmd  pb_cmd,
                    std::string& msg,
                    int timeout,
                    EcReplFault &fault);


    void zmq_cmd_recv(std::string& msg,
                      iit::advr::CmdType cmd_sent,
                      zmq::socket_t& socket,
                      EcReplFault &fault);



    bool zmq_cmd_send(iit::advr::Repl_cmd  pb_cmd,
                      zmq::socket_t& socket,
                      EcReplFault &fault
                     );
};

#endif

