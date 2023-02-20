#ifndef __COMMON__
#define __COMMON__


#include "ec_client_cmd.h"

#include "ec_msgs/SlaveCmdInfo.h"
#include "ec_msgs/CmdStatus.h"

    
    /**
    * @brief Common Method to select the slaves from the internal list of the slaves or from an external request.
    * The method checks if the slave requested vector is empty.
    * @param internal_list_slaves p_internal_list_slaves: List of internal slaves.
    * @param slaves_requested p_slaves_requested: Slave requested.
    * @param slaves_selected p_slaves_selected: Slave Selected.
    * @param slaves_cmd_status p_slaves_cmd_status: Message of the command status. 
    * @return bool
    */
    static inline bool select_slaves(std::vector<std::string> internal_list_slaves,
                                 std::vector<std::string> slaves_requested,
                                 std::vector<std::string> &slaves_selected,
                                 ec_msgs::CmdStatus &slaves_cmd_status)
{
    bool ret=false;
    
    if(slaves_requested.size()!=0)
    {
        std::string slave_name=slaves_requested[0];
        if((slave_name=="all")||(slave_name==""))
        {
            slaves_selected.resize(internal_list_slaves.size());
            slaves_selected=internal_list_slaves;
        }
        else
        {
            slaves_selected.resize(slaves_requested.size());
            slaves_selected=slaves_requested;
        }
        ret=true;
    }
    else
    {
        slaves_cmd_status.status="No Slaves Selected";
        slaves_cmd_status.success_slaves.push_back("NULL");
        slaves_cmd_status.unsuccess_slaves.push_back("NULL");
        
    } 
    
    return ret;
}

template <typename T1,typename T2>
static inline bool slaves_request_adpating(std::vector<std::string> slaves_requested,
                                           std::vector<std::string> slaves_selected,
                                           std::vector<T1> req,
                                           std::vector<T2> &req_adpted,
                                           ec_msgs::CmdStatus &slaves_cmd_status)               
{
    bool ret=false;
    if(slaves_requested.size()!=0 || slaves_selected.size()!=0)
    {
        std::string slave_name=slaves_requested[0];
        if((slave_name=="all")||(slave_name==""))
        {
            req_adpted.assign(slaves_selected.size(), req[0]);
        }
        else
        {
            req_adpted.resize(req.size());
            for(int i=0; i<req_adpted.size();i++)
            {
                req_adpted[i]=req[i];    
            }
        }
        ret=true;
    }
    else
    {
        slaves_cmd_status.status="No Slaves Selected";
        slaves_cmd_status.success_slaves.push_back("NULL");
        slaves_cmd_status.unsuccess_slaves.push_back("NULL");
        
    }
    
    return ret;
}                            


/**
* @brief Common method to check the feedback from EtherCAT Master Server.
* 
* @param fault p_fault: Fault object coming from the ZMQ communication.
* @param sent_cmd p_sent_cmd: Command sent to the EtherCAT Master Server.
* @param slave_name p_slave_name: Slave Name.
* @param cmd p_cmd: ZMQ Command
* @param msg p_msg: Feedback message.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...)
* @param unsuccess_slaves p_unsuccess_slaves: Unsuccess Slaves.
* @param success_slaves p_success_slaves: Success Slaves.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void feedback_cmd(EC_Client_CMD_Fault fault,
                                bool sent_cmd,
                                std::string slave_name,
                                std::string cmd,
                                std::string msg,
                                std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                std::vector<std::string> &unsuccess_slaves,
                                std::vector<std::string> &success_slaves,
                                bool &status_callback)
{

    ec_msgs::SlaveCmdInfo slave_cmd_info;
    slave_cmd_info.slave_name=slave_name;
    slave_cmd_info.cmd=cmd;
    
    if(sent_cmd)
    {
        slave_cmd_info.fault_cmd= fault.get_zmq_cmd();
        slave_cmd_info.fault_info = fault.get_info();
        slave_cmd_info.fault_recovery = fault.get_recovery_info();
        slave_cmd_info.msg=msg;

        if(fault.get_type()!=EC_CLIENT_CMD_STATUS::OK)
        {
            
            slave_cmd_info.status="FAULT";
            unsuccess_slaves.push_back(slave_name);
            
            if(fault.get_type()==EC_CLIENT_CMD_STATUS::TIMEOUT)
            {
                status_callback=false;
                return;
            }
            
        }
        else
        {
            success_slaves.push_back(slave_name);
            
            std::string status="Performed operation on the slave: "+slave_name;
            slave_cmd_info.status=status;
        }
    
    }
    else
    {
        unsuccess_slaves.push_back(slave_name);
        
        std::string status="Slave Name: "+slave_name+" doesn't exist for this robot";
        slave_cmd_info.status=status;
    }
    
    slaves_cmd_info.push_back(slave_cmd_info);

}

/**
* @brief Common Method to check which slave has a success or unsuccess feedback from zmq command.
* 
* @param slaves_selected p_slaves_selected: Slave selected.
* @param cmd p_cmd: ZMQ command type.
* @param unsuccess_cmd_slaves p_unsuccess_cmd_slaves: Unsuccess Slaves.
* @param success_cmd_slaves p_success_cmd_slaves: Success Slaves.
* @param slaves_cmd_status p_slaves_cmd_status: Message to send through the services.
*/
static inline void check_slaves_cmd_status(std::vector<std::string> slaves_selected,
                                           std::string cmd,
                                           std::vector<std::string> unsuccess_cmd_slaves,
                                           std::vector<std::string> success_cmd_slaves,
                                           ec_msgs::CmdStatus &slaves_cmd_status)
{
    if(success_cmd_slaves.size()!=slaves_selected.size())
    {
        if(success_cmd_slaves.size()==0)
        {
            slaves_cmd_status.status="NO Slaves requested performed the command !";
            slaves_cmd_status.success_slaves.push_back("NONE");
            slaves_cmd_status.unsuccess_slaves.push_back("ALL");
        }
        else
        {    
            std::string status ="Launched " + cmd + " command not for all slaves requested";
            slaves_cmd_status.status=status;
            for (int i=0; i< success_cmd_slaves.size();i++)
            {
                slaves_cmd_status.success_slaves.push_back(success_cmd_slaves[i]);
            }  
            
            for (int i=0; i< unsuccess_cmd_slaves.size();i++)
            {
                slaves_cmd_status.unsuccess_slaves.push_back(unsuccess_cmd_slaves[i]);
            }      
        }
    }
    else
    {
        if(!slaves_selected.empty())
        {
            std::string status="Launched  "+cmd+" command for all slaves requested successfully";
            slaves_cmd_status.status=status;
            slaves_cmd_status.success_slaves.push_back("ALL");
            slaves_cmd_status.unsuccess_slaves.push_back("NONE");
        }
        else
        {
            slaves_cmd_status.status="NO Slaves requested performed the command !";
            slaves_cmd_status.success_slaves.push_back("NONE");
            slaves_cmd_status.unsuccess_slaves.push_back("ALL");
        }
    }
}

#endif
