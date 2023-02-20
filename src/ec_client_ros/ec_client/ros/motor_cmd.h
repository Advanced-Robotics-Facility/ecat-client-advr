#ifndef __MOTOR_CMD__
#define __MOTOR_CMD__

#include "common_cmd.h"
#include <chrono>
#include <thread>

/**************** CTRL CMD *//////////////////////////


/**
* @brief This method is responsible to send a control command to the motors.
* 
* @param slaves_selected p_slaves_selected: Slaves Selected.
* @param motor_map_setup p_motor_map_setup: Motor Setup Map.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command Type.
* @param type p_type: Control Command Type.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void ctrl_cmd(std::vector<std::string> slaves_selected,
                            std::map<std::string, Motor_Setup::Ptr> motor_map_setup,
                            EC_Client_CMD::Ptr ec_client_cmd,
                            std::string cmd,
                            iit::advr::Ctrl_cmd_Type type,
                            std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                            ec_msgs::CmdStatus &slaves_cmd_status,
                            bool &status_callback)
{
    
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!motor_map_setup.empty())
        {
            if(motor_map_setup.count(slaves_selected[i])>0)
            {
                Motor_Setup::Ptr motor_setup=motor_map_setup.at(slaves_selected[i]);
                
                int motor_id=motor_setup->get_id_low_level();
                float value=0;
                std::vector<float> gains;
                
                if((type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START)||
                (type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS))
                    
                {
                    
                    if(motor_setup->get_control_mode()=="3B_motor_pos_ctrl")
                    {
                        value=0x3B;
                        gains=motor_setup->get_position_gains();
                    }
                    else if(motor_setup->get_control_mode()=="71_motor_vel_ctrl")
                    {
                        value=0x71;
                        gains=motor_setup->get_velocity_gains();
                    }
                    else if(motor_setup->get_control_mode()=="D4_impedance_ctrl")
                    {
                        value=0xD4;
                        gains=motor_setup->get_impedance_gains();
                    }
                    else
                    {
                        value=0x3B;
                        gains=motor_setup->get_position_gains();
                    }
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_FAN)
                {
                    if(motor_setup->get_fan_onoff())
                    {
                        value=1.0;
                    }
                    else
                    {
                        value=0.0;
                    }
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_LED)
                {
                    if(motor_setup->get_led_onoff())
                    {
                        value=1.0;
                    }
                    else
                    {
                        value=0.0;
                    }
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_POSITION)
                {
                    value=motor_setup->get_actual_position();
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_VELOCITY)
                {
                    value=motor_setup->get_actual_velocity();
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_TORQUE)
                {
                    value=motor_setup->get_actual_torque();
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_CURRENT)
                {
                    value=motor_setup->get_actual_amperage();
                }
                else if(type==iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_HOME)
                {
                    value=motor_setup->get_homing_position();
                }
                
                ec_client_cmd->Ctrl_cmd(type,motor_id,value,gains,msg);
                
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


/**************** TRJ CMD *//////////////////////////

/**
* @brief This method is responsible to create a trajectory for the motors.
* 
* @param slaves_selected p_slaves_selected: Slaves Selected.
* @param motor_map_setup p_motor_map_setup: Motor Setup Map.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command Type.
* @param type p_type: Trajectory Type.
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void trajectory_cmd(std::vector<std::string> slaves_selected,
                                  std::map<std::string, Motor_Setup::Ptr> motor_map_setup,
                                  EC_Client_CMD::Ptr ec_client_cmd,
                                  std::string cmd,
                                  iit::advr::Trajectory_cmd_Type type,
                                  std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                  ec_msgs::CmdStatus &slaves_cmd_status,
                                  bool &status_callback)
{
    
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!motor_map_setup.empty())
        {
            if(motor_map_setup.count(slaves_selected[i])>0)
            {
                Motor_Setup::Ptr motor_setup=motor_map_setup.at(slaves_selected[i]);
                
                int motor_id=motor_setup->get_id_low_level();
                Motor_Setup::trajectoy_t trajectoy_s=motor_setup->get_trajectory_structure();
                std::string trj_name=trajectoy_s.trj_name;
                
                EC_Client_CMD::homing_par_t homing_par;
                EC_Client_CMD::period_par_t period_par;
                EC_Client_CMD::smooth_par_t smooth_par;
                
                if(type==iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_HOMING)
                {
                    homing_par.x.resize(trajectoy_s.homing_x.size());
                    homing_par.x=trajectoy_s.homing_x;
                }
                else if(type==iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_SINE)
                {
                    period_par.freq=trajectoy_s.period_freq;
                    period_par.ampl=trajectoy_s.period_ampl;
                    period_par.teta=trajectoy_s.period_teta;
                    period_par.secs=trajectoy_s.period_secs;
                }
                else if(type==iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_SMOOTHER)
                {
                    smooth_par.x.resize(trajectoy_s.smooth_x.size());
                    smooth_par.x=trajectoy_s.smooth_x;
                    
                    smooth_par.y.resize(trajectoy_s.smooth_y.size());
                    smooth_par.y=trajectoy_s.smooth_y;
                }
                ec_client_cmd->Trajectory_Cmd(type,trj_name,motor_id,homing_par,period_par,smooth_par,msg);
                
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



/**************** TRJ QUEUE CMD *//////////////////////////

/**
* @brief ...
* 
* @param slaves_selected p_slaves_selected: Slaves Selected.
* @param motor_map_setup p_motor_map_setup: Motor Map setup.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command Type. 
* @param type p_type: Trajectory queue (start/clear) command
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void trj_queue_cmd(std::vector<std::string> slaves_selected,
                                 std::map<std::string, Motor_Setup::Ptr> motor_map_setup,
                                 EC_Client_CMD::Ptr ec_client_cmd,
                                 std::string cmd,
                                 iit::advr::Trj_queue_cmd_Type type,
                                 std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                 ec_msgs::CmdStatus &slaves_cmd_status,
                                 bool &status_callback)
{
    
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!motor_map_setup.empty())
        {
            if(motor_map_setup.count(slaves_selected[i])>0)
            {
                Motor_Setup::Ptr motor_setup=motor_map_setup.at(slaves_selected[i]);
                
                Motor_Setup::trajectoy_t trajectoy_s=motor_setup->get_trajectory_structure();
                
                std::vector<std::string> trj_names;
                trj_names.push_back(trajectoy_s.trj_name);
                
                ec_client_cmd->Trj_queue_cmd(type,trj_names,msg);
                
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

/**************** PDOs_aux_cmd *//////////////////////////

/**
* @brief ...
* 
* @param slaves_selected p_slaves_selected: Slaves Selected.
* @param motor_map_setup p_motor_map_setup: Motor Map setup.
* @param ec_client_cmd p_ec_client_cmd: EC EC Client CMD smart pointer to send zmq command and receive the reply.
* @param cmd p_cmd: Command Type. 
* @param type p_type: Trajectory queue (start/clear) command
* @param slaves_cmd_info p_slaves_cmd_info: Vector of slaves command information (Fault, Status, Command , Feedback Message, etc...).
* @param slaves_cmd_status p_slaves_cmd_status: Message of the command status.
* @param status_callback p_status_callback: Return the status of the zmq command.
*/
static inline void PDOs_aux_cmd(std::vector<std::string> slaves_selected,
                                 std::map<std::string, Motor_Setup::Ptr> motor_map_setup,
                                 EC_Client_CMD::Ptr ec_client_cmd,
                                 std::string cmd,
                                 iit::advr::PDOs_aux_cmd_Aux_cmd_Type type,
                                 std::vector<ec_msgs::SlaveCmdInfo> &slaves_cmd_info,
                                 ec_msgs::CmdStatus &slaves_cmd_status,
                                 bool &status_callback)
{
    
    std::vector<std::string> success_cmd_slaves,unsuccess_cmd_slaves;
    for(int i=0; i<slaves_selected.size() && status_callback;i++)
    {
        std::string msg="";
        bool sent_cmd=false;
        if(!motor_map_setup.empty())
        {
            EC_Client_CMD_Fault cmd_fault;
            if(motor_map_setup.count(slaves_selected[i])>0)
            {
                Motor_Setup::Ptr motor_setup=motor_map_setup.at(slaves_selected[i]);
                
                bool brake_present = motor_setup->get_brake_present();
                if(brake_present)
                {
                    EC_Client_CMD::aux_cmd_message_t aux_cmd;
                    std::vector<EC_Client_CMD::aux_cmd_message_t> aux_cmds;
                    
                    // prepare message for releasing or engaging the brake
                    int slave_id =motor_setup->get_id_low_level();
                    aux_cmd.board_id = slave_id;
                    aux_cmd.type = type;
                    aux_cmds.push_back(aux_cmd);

                    ec_client_cmd->PDOs_aux_cmd(aux_cmds,msg);
                    
                    sent_cmd=true;
                    cmd_fault = ec_client_cmd->get_fault();
                    if(cmd_fault.get_type()==EC_CLIENT_CMD_STATUS::OK)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        // verify the brake status
                        std::vector<std::string> rd_sdo={"master_cmd_status_copy"}; 
                        
                        msg.clear();
                        //send message
                        ec_client_cmd->Slave_SDO_cmd(slave_id, 
                                                    rd_sdo,
                                                    {},  // ignored
                                                    msg);
                        
                        cmd_fault = ec_client_cmd->get_fault();
                        if(cmd_fault.get_type()==EC_CLIENT_CMD_STATUS::OK)
                        {
                            // get brake status in double
                            auto n = YAML::Load(msg);
                            uint32_t brake_sts=0;
                            if(auto read_master_cmd_status = n[rd_sdo[0]])
                            {
                                int master_cmd_status = read_master_cmd_status.as<int>();
                                brake_sts = master_cmd_status & 3 ; //00 unknown
                                                                    //01 release brake 
                                                                    //10 enganged brake
                                                                    //11 error   
                                if(brake_sts != type)
                                {
                                    cmd_fault.set_type(EC_CLIENT_CMD_STATUS::WRONG_FB_MSG);
                                    cmd_fault.set_info("brake status: " + std::to_string(brake_sts) + 
                                                    " different to the command request: " + std::to_string(type));
                                    cmd_fault.set_recovery_info("Update the slave firmware");
                                }
                            }
                            else
                            {
                                cmd_fault.set_type(EC_CLIENT_CMD_STATUS::WRONG_FB_MSG);
                                cmd_fault.set_info("master_cmd_status_copy doesn't exist");
                                cmd_fault.set_recovery_info("Update the slave firmware");
                            }
                            
                        }
                    }
                }
                else
                {
                    cmd_fault.set_type(EC_CLIENT_CMD_STATUS::NACK);
                    cmd_fault.set_info("this motor hasn't the brake");
                    cmd_fault.set_recovery_info("Avoid the request to release or engage the brake on this motor");
                }
            }

            feedback_cmd(cmd_fault,
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


#endif
