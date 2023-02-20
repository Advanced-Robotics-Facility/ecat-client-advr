#ifndef __SLAVE_CMD__
#define __SLAVE_CMD__

#include "common_cmd.h"

#include <fstream>

/**************** SLAVE SDO CMD *//////////////////////////

/**
* @brief Method to read or write the SDO.
* 
* @param slaves_selected p_slaves_selected: Slaves selected.
* @param map_id_slaves p_map_id_slaves: ID Map of the slaves.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command type.
* @param rd_sdo p_rd_sdo Vector of reading SDO.
* @param wr_sdo p_wr_sdo: Key-Value Map for writing SDO.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void slave_sdo_cmd(std::vector<std::string> slaves_selected,
                                 std::map<std::string,int> map_id_slaves,
                                 EC_Client_CMD::Ptr ec_client_cmd,
                                 std::string cmd,
                                 std::vector<std::string> rd_sdo,
                                 std::map<std::string ,std::string> wr_sdo,
                                 std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                 ec_msgs::CmdStatus &slaves_cmd_status,
                                 bool &status_callback)
{
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback ;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!map_id_slaves.empty())
        {
            if(map_id_slaves.count(slaves_selected[i])>0)
            {
                int slave_id=map_id_slaves[slaves_selected[i]];
                
                ec_client_cmd->Slave_SDO_cmd(slave_id,rd_sdo,wr_sdo,msg);
                sent_cmd=true;
        
            }
            EC_Client_CMD_Fault fault(ec_client_cmd->get_fault());
            
            feedback_cmd(fault,
                        sent_cmd,
                        slaves_selected[i],
                        cmd,
                        msg,
                        slaves_cmd_info,
                        unsuccess_cmd_slaves,
                        success_cmd_slaves,
                        status_callback
                        );
        }
    }
    
    check_slaves_cmd_status(slaves_selected,
                            cmd,
                            unsuccess_cmd_slaves,
                            success_cmd_slaves,
                            slaves_cmd_status);
}

/**************** Slave SDO INFO *//////////////////////////

/**
* @brief ...
* 
* @param slaves_selected p_slaves_selected: Slaves Selected.
* @param map_id_slaves p_map_id_slaves: ID Map of the slaves.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command Type.
* @param type p_type: SDO information type (NAME or OBJD).
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void slave_sdo_info(std::vector<std::string> slaves_selected,
                                 std::map<std::string,int> map_id_slaves,
                                 EC_Client_CMD::Ptr ec_client_cmd,
                                 std::string cmd,
                                 iit::advr::Slave_SDO_info_Type type,
                                 std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                 ec_msgs::CmdStatus &slaves_cmd_status,
                                 bool &status_callback)
{
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback ;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!map_id_slaves.empty())
        {
            if(map_id_slaves.count(slaves_selected[i])>0)
            {
                int slave_id=map_id_slaves[slaves_selected[i]];
                
                ec_client_cmd->Slave_SDO_info(type,
                                              slave_id,
                                              msg);
                sent_cmd=true;
        
            }
            EC_Client_CMD_Fault fault(ec_client_cmd->get_fault());
            
            feedback_cmd(fault,
                        sent_cmd,
                        slaves_selected[i],
                        cmd,
                        msg,
                        slaves_cmd_info,
                        unsuccess_cmd_slaves,
                        success_cmd_slaves,
                        status_callback
                        );
        }
    }
    
    check_slaves_cmd_status(slaves_selected,
                            cmd,
                            unsuccess_cmd_slaves,
                            success_cmd_slaves,
                            slaves_cmd_status);
}


/**************** Flash CMD *//////////////////////////

/**
* @brief Method used to save, load or load from default the parameters of slaves to/from the flash.
* 
* @param slaves_selected p_slaves_selected: Slave Selected.
* @param map_id_slaves p_map_id_slaves: ID Map of the slaves.
* @param ec_client_cmd p_ec_client_cmd:  EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command type.
* @param type p_type: Flash command type.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void flash_cmd(std::vector<std::string> slaves_selected,
                             std::map<std::string,int> map_id_slaves,
                             EC_Client_CMD::Ptr ec_client_cmd,
                             std::string cmd,
                             iit::advr::Flash_cmd_Type type,
                             std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                             ec_msgs::CmdStatus &slaves_cmd_status,
                             bool &status_callback)
{
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback ;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!map_id_slaves.empty())
        {
            if(map_id_slaves.count(slaves_selected[i])>0)
            {
                int slave_id=map_id_slaves[slaves_selected[i]];
                
                ec_client_cmd->Flash_cmd(type,
                                         slave_id,
                                         msg);
                sent_cmd=true;
        
            }
            EC_Client_CMD_Fault fault(ec_client_cmd->get_fault());
            
            feedback_cmd(fault,
                        sent_cmd,
                        slaves_selected[i],
                        cmd,
                        msg,
                        slaves_cmd_info,
                        unsuccess_cmd_slaves,
                        success_cmd_slaves,
                        status_callback
                        );
        }
    }
    
    check_slaves_cmd_status(slaves_selected,
                            cmd,
                            unsuccess_cmd_slaves,
                            success_cmd_slaves,
                            slaves_cmd_status);
}


/**************** FOE Master *//////////////////////////

/**
* @brief Method used to Flash new firmware into the slaves.
* 
* @param slaves_selected p_slaves_selected: Slave Selected.
* @param map_id_slaves p_map_id_slaves: ID Map of the slaves.
* @param ec_client_cmd p_ec_client_cmd:  EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command type.
* @param foe_info p_foe_info: c28 and m3 filenames, c28 and m3 passwords.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
* @return bool
*/
static inline bool foe_master(std::vector<std::string> slaves_selected,
                              std::map<std::string,int> map_id_slaves,
                              EC_Client_CMD::Ptr ec_client_cmd,
                              std::string cmd,
                              std::vector<std::string> foe_info,
                              std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                              ec_msgs::CmdStatus &slaves_cmd_status,
                              bool &status_callback)
{
    
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback ;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!map_id_slaves.empty())
        {
            if(map_id_slaves.count(slaves_selected[i])>0)
            {
                int slave_id=map_id_slaves[slaves_selected[i]];
                
                if((foe_info[0]!="")&&(foe_info[1]!="")&&(foe_info[2]!="")&&(foe_info[3]!=""))
                {
                    std::string m3_file=foe_info[0];
                    std::string m3_password=foe_info[1];
                    
                    std::string c28_file=foe_info[0];
                    std::string c28_password=foe_info[1];
                    
                    std::stringstream m3_password_stream,c28_password_stream;
                    unsigned long int m3_password_numb,c28_password_numb;
                    
                    m3_password_stream << m3_password;
                    m3_password_stream >> std::hex >>m3_password_numb;
                    
                    c28_password_stream << c28_password;
                    c28_password_stream >> std::hex >>c28_password_numb;
                    
                    ec_client_cmd->FOE_Master(m3_file,
                                              m3_password_numb,
                                              "m3",
                                              0,
                                              slave_id,
                                              msg);
                    
                    EC_Client_CMD_Fault fault(ec_client_cmd->get_fault());

                    if(fault.get_type()!=EC_CLIENT_CMD_STATUS::OK)
                    {
                        ec_client_cmd->FOE_Master(c28_file,
                                                  c28_password_numb,
                                                  "c28",
                                                  0,
                                                  slave_id,
                                                  msg);
                        
                        sent_cmd=true;
                    }
                    
                }
                else
                    return false;
                
            }
            
            EC_Client_CMD_Fault fault(ec_client_cmd->get_fault());
            
            feedback_cmd(fault,
                        sent_cmd,
                        slaves_selected[i],
                        cmd,
                        msg,
                        slaves_cmd_info,
                        unsuccess_cmd_slaves,
                        success_cmd_slaves,
                        status_callback
                        );
        }
    }
    
    check_slaves_cmd_status(slaves_selected,
                            cmd,
                            unsuccess_cmd_slaves,
                            success_cmd_slaves,
                            slaves_cmd_status);
    
    return true;
    
}



#endif
