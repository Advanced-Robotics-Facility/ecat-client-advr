#include "motor_ros.h"
#include "slave_cmd.h"
#include "motor_cmd.h"

#include <esc/mc_centAC_esc.h>


using namespace std;

Motor_ROS::Motor_ROS(std::shared_ptr<ros::NodeHandle> node):
_node(node)
{
    _status_callback=true;
    
            
    _pub_motor_pdo=_node->advertise<ec_msgs::MotorPDO>("motorPDO", 1);

    _servie_print_motors_info                   =   _node->advertiseService("print_motors_info", &Motor_ROS::print_motors_info, this);
    
    _service_stop_motors                        =   _node->advertiseService("stop_motors", &Motor_ROS::stop_motors, this);
    _service_start_motors_posmode               =   _node->advertiseService("start_motors_posmode", &Motor_ROS::start_motors_posmode, this);
    _service_start_motors_velmode               =   _node->advertiseService("start_motors_velmode", &Motor_ROS::start_motors_velmode, this);
    _service_start_motors_impmode               =   _node->advertiseService("start_motors_impmode", &Motor_ROS::start_motors_impmode, this);
    _service_release_motors_brake               =   _node->advertiseService("release_motors_brake", &Motor_ROS::release_motors_brake, this);
    _service_engage_motors_brake                =   _node->advertiseService("engage_motors_brake", &Motor_ROS::engage_motors_brake, this);
    
    
    _service_get_motors_ampere_info             =   _node->advertiseService("get_motors_ampere_info", &Motor_ROS::get_motors_ampere_info, this);
    _service_get_motors_limits_info             =   _node->advertiseService("get_motors_limits_info", &Motor_ROS::get_motors_limits_info, this);
    _service_get_motors_gains_info              =   _node->advertiseService("get_motors_gains_info", &Motor_ROS::get_motors_gains_info, this);
        

    _service_set_motors_posgains                 =   _node->advertiseService("set_motors_posgains", &Motor_ROS::set_motors_posgains, this);
    _service_set_motors_velgains                 =   _node->advertiseService("set_motors_velgains", &Motor_ROS::set_motors_velgains, this);
    _service_set_motors_impgains                 =   _node->advertiseService("set_motors_impgains", &Motor_ROS::set_motors_impgains, this);
    _service_set_motors_limits                   =   _node->advertiseService("set_motors_limits", &Motor_ROS::set_motors_limits, this);
    _service_set_motors_max_ampere               =   _node->advertiseService("set_motors_max_ampere", &Motor_ROS::set_motors_max_ampere, this);

    _service_set_motors_position                 =   _node->advertiseService("set_motors_position", &Motor_ROS::set_motors_position, this);
    _service_set_motors_velocity                 =   _node->advertiseService("set_motors_velocity", &Motor_ROS::set_motors_velocity, this);
    _service_set_motors_torque                   =   _node->advertiseService("set_motors_torque", &Motor_ROS::set_motors_torque, this);
    _service_set_motors_amperage                 =   _node->advertiseService("set_motors_amperage", &Motor_ROS::set_motors_amperage, this);
    _service_set_motors_homing_position          =   _node->advertiseService("set_motors_homing_position", &Motor_ROS::set_motors_homing_position, this);
    
    _service_set_motors_fan                      =   _node->advertiseService("set_motors_fan", &Motor_ROS::set_motors_fan, this);
    _service_set_motors_led                      =   _node->advertiseService("set_motors_led", &Motor_ROS::set_motors_led, this);

    _service_set_motor_homing_trajectory         =   _node->advertiseService("set_motor_homing_trajectory", &Motor_ROS::set_motor_homing_trajectory, this);
    _service_set_motor_period_trajectory         =   _node->advertiseService("set_motor_period_trajectory", &Motor_ROS::set_motor_period_trajectory, this);
    _service_set_motor_smooth_trajectory         =   _node->advertiseService("set_motor_smooth_trajectory", &Motor_ROS::set_motor_smooth_trajectory, this);
    
    
    _service_start_motors_trajectory             =   _node->advertiseService("start_motors_trajectory", &Motor_ROS::start_motors_trajectory, this);
    _service_clear_motors_trajectory             =   _node->advertiseService("clear_motors_trajectory", &Motor_ROS::clear_motors_trajectory, this);

}


void Motor_ROS::set_ec_client_cmd(EC_Client_CMD::Ptr ec_client_cmd)
{   
    _ec_client_cmd.reset();
    _ec_client_cmd=ec_client_cmd;
    
}

void Motor_ROS::set_status_callback(bool status_callback)
{
    _status_callback=status_callback;
}

bool Motor_ROS::get_status_callback()
{
    return _status_callback;
}

void Motor_ROS::set_xbotcorepresence(bool xbotcorepresence)
{
    _xbotcorepresence=xbotcorepresence;
}



std::map<std::string, Motor_Setup::Ptr> Motor_ROS::get_motor_map_setup()
{
    return _motor_map_setup;
}

void Motor_ROS::set_motor_map_setup(std::map<std::string, Motor_Setup::Ptr> motor_map_setup)
{
    if(!_motor_map_setup.empty())
        _motor_map_setup.clear();
    
    _motor_map_setup=motor_map_setup;
    
    if(!_internal_list_motors.empty())
        _internal_list_motors.clear();
    
    if(!_id_map_motors.empty())
        _id_map_motors.clear();
    
    for(auto [motor_name,motor_setup]:_motor_map_setup)
    {
        _internal_list_motors.push_back(motor_name);
        _id_map_motors[motor_name]=motor_setup->get_id_low_level();
    }
}


std::map<std::string,EC_Client_PDO::Ptr> Motor_ROS::get_ec_client_pdo_map()
{
    return _ec_client_pdo_map;
}

void Motor_ROS::set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr>ec_client_pdo_map)
{
    if(!_ec_client_pdo_map.empty())
        _ec_client_pdo_map.clear();

    _ec_client_pdo_map=ec_client_pdo_map;
}


void Motor_ROS::set_model_interface(XBot::ModelInterface::Ptr model)
{
    _model=model;
}
    
void Motor_ROS::set_robotviz(std::shared_ptr<RobotViz> robotviz)
{
    _robotviz=robotviz;
}

void Motor_ROS::clear_joint_map()
{
    if(!_q.empty())
        _q.clear();
    
    for(auto [motor_name,motor_setup]:_motor_map_setup)
    {
        _q[motor_name]=0.0;
    }
}

void Motor_ROS::clear_motors_pdo_message()
{
    _motors_pdo.slave_name.clear();
    _motors_pdo.slave_name.resize(_motor_map_setup.size());
        
    _motors_pdo.motor_pos.clear();
    _motors_pdo.motor_pos.resize(_motor_map_setup.size());
    _motors_pdo.motor_vel.clear();
    _motors_pdo.motor_vel.resize(_motor_map_setup.size());
    
    _motors_pdo.link_pos.clear();
    _motors_pdo.link_pos.resize(_motor_map_setup.size());
    _motors_pdo.link_vel.clear();
    _motors_pdo.link_vel.resize(_motor_map_setup.size());


    _motors_pdo.torque.clear();
    _motors_pdo.torque.resize(_motor_map_setup.size());

    _motors_pdo.motor_temp.clear();
    _motors_pdo.motor_temp.resize(_motor_map_setup.size());
    _motors_pdo.board_temp.clear();
    _motors_pdo.board_temp.resize(_motor_map_setup.size());

    _motors_pdo.fault.clear();
    _motors_pdo.fault.resize(_motor_map_setup.size());
    
    _motors_pdo.fault_numb.clear();
    _motors_pdo.fault_numb.resize(_motor_map_setup.size());

    _motors_pdo.pos_ref.clear();
    _motors_pdo.pos_ref.resize(_motor_map_setup.size());
    _motors_pdo.vel_ref.clear();
    _motors_pdo.vel_ref.resize(_motor_map_setup.size());
    _motors_pdo.tor_ref.clear();
    _motors_pdo.tor_ref.resize(_motor_map_setup.size());
}

void Motor_ROS::pub_robot()
{
    if(_model)
    {
        _robotviz->publishMarkers(ros::Time::now(),std::vector<std::string>());
    }
}

#define HANDLE_FAULT(type,fault_msg) \
    if(cent_fault.bit.type) \
{ \
    fault_msg += #type; \
    fault_msg += ';'; \
}

std::string Motor_ROS::handle_fault(uint32_t fault)
{
    
    string fault_msg;
    iit::ecat::centAC_fault_t cent_fault;
    cent_fault.all=fault;
    
    HANDLE_FAULT(m3_rxpdo_pos_ref,fault_msg);
    HANDLE_FAULT(m3_rxpdo_vel_ref,fault_msg);
    HANDLE_FAULT(m3_rxpdo_tor_ref,fault_msg);
    HANDLE_FAULT(m3_fault_hardware,fault_msg);

    HANDLE_FAULT(m3_params_out_of_range,fault_msg);
    HANDLE_FAULT(m3_torque_array_not_loaded,fault_msg);
    HANDLE_FAULT(m3_torque_read_error,fault_msg);
    HANDLE_FAULT(m3_out_of_limits,fault_msg);

    HANDLE_FAULT(m3_link_enc_error,fault_msg);
    HANDLE_FAULT(m3_defl_enc_error,fault_msg);
    HANDLE_FAULT(m3_temperature_warning,fault_msg);
    HANDLE_FAULT(m3_temperature_error,fault_msg);
    HANDLE_FAULT(m3_safety_override,fault_msg);

    HANDLE_FAULT(c28_motor_enc_error,fault_msg);
    HANDLE_FAULT(c28_v_batt_read_fault,fault_msg);
    HANDLE_FAULT(c28_enter_sand_box,fault_msg);

    if(!fault_msg.empty())
    {
        fault_msg.resize(fault_msg.size() - 1);  // remove trailing ';'
    }
    
    return fault_msg;
                    
}

bool Motor_ROS::pub_motors_pdo()
{
    int i=0;
    for(auto [motor_name,motor_setup]:_motor_map_setup)
    {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        string msg="";

        if(_ec_client_pdo_map.count(motor_name)>0)
        {
            _motors_pdo.slave_name[i]=motor_name;
            EC_Client_PDO::Ptr ec_client_pdo=_ec_client_pdo_map.at(motor_name);
            if(ec_client_pdo->zmq_cmd_recv_pdo(msg,pb_rx_pdos))
            {
                if(msg!="")
                {
                    _motors_pdo.motor_pos[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
                    _motors_pdo.motor_vel[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
                    
                    _motors_pdo.link_pos[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
                    _motors_pdo.link_vel[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
                    
                    _motors_pdo.torque[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
                    
                    _motors_pdo.motor_temp[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
                    _motors_pdo.board_temp[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();
                    
                    string fault_msg=handle_fault(pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault());
                    _motors_pdo.fault[i]=fault_msg;
                    _motors_pdo.fault_numb[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
                    
                    _motors_pdo.pos_ref[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
                    _motors_pdo.vel_ref[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
                    _motors_pdo.tor_ref[i]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();

                    _q[motor_name]=pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
                    
                    _model->setJointPosition(_q);
                    _model->update();
                }
            }
            else
            {
                std::cout << "Error!" << std::endl;
                return false;
            }
        
        }
        i++;
    }
    
    _pub_motor_pdo.publish(_motors_pdo);
    
    return true;
}


void Motor_ROS::prepare_motor_message(string slave_name,Motor_Setup::Ptr motor_setup,vector<ec_msgs::SlaveDescription> &slaves_descr)
{
    ec_msgs::SlaveDescription slave_descr;
    slave_descr.status="SLAVE FOUND";
    slave_descr.slave_name=slave_name;
    slave_descr.id_low_level=to_string(motor_setup->get_id_low_level());
    slave_descr.control_mode=motor_setup->get_control_mode();
    slave_descr.sign=motor_setup->get_sign();
    slave_descr.offset=motor_setup->get_pos_offset();
    slave_descr.max_current=motor_setup->get_max_current_A();
    auto position_gains=motor_setup->get_position_gains();
    slave_descr.position_gain=position_gains;

    auto velocity_gains=motor_setup->get_velocity_gains();
    slave_descr.velocity_gain=velocity_gains;

    auto impedance_gains=motor_setup->get_impedance_gains();
    slave_descr.impedance_gain=impedance_gains;
    
    slave_descr.home_position=motor_setup->get_homing_position();
    
    slaves_descr.push_back(slave_descr);
}

bool Motor_ROS::print_motors_info(ec_srvs::PrintMotorInfo::Request  &req,ec_srvs::PrintMotorInfo::Response &res)
{
    _status_callback=true;
    
    if(req.slave_name.size()!=0)
    {
        string slave_name=req.slave_name[0];
        if((slave_name=="all")||(slave_name==""))
        {
            for(auto [slave_name_int,motor_setup]:_motor_map_setup)
            {
                
                prepare_motor_message(slave_name_int,motor_setup,res.slaves_descr);
            }
        }
        else
        {
            for(int i=0 ; i<req.slave_name.size();i++)
            {
                string slave_name_int=req.slave_name[0];
                if(_motor_map_setup.count(slave_name)>0)
                {
                    auto motor_setup=_motor_map_setup.at(slave_name_int);
                    prepare_motor_message(slave_name_int,motor_setup,res.slaves_descr);
                }
                else
                {
                    ec_msgs::SlaveDescription slave_descr;
                    string status="Slave Name: "+slave_name+"doesn't exist for this robot";
                    slave_descr.status=status;
                    res.slaves_descr.push_back(slave_descr);
                }
            }
        }
    }
    
    return true;
}



bool Motor_ROS::get_motors_ampere_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
   _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    std::vector<std::string> rd_sdo;
    std::map<std::string ,std::string> wr_sdo;
    
    std::string cmd="get_motors_ampere_info";
    
    
    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        rd_sdo.push_back("Max_curr");
        rd_sdo.push_back("CurrGainP");
        rd_sdo.push_back("CurrGainI");
        rd_sdo.push_back("CurrGainD");
        
        slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    
    return true;
}



bool Motor_ROS::get_motors_limits_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    std::vector<std::string> rd_sdo;
    std::map<std::string ,std::string> wr_sdo;
    
    std::string cmd="get_motors_limits_info";
    
    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        rd_sdo.push_back("Min_pos");
        rd_sdo.push_back("Max_pos");
        rd_sdo.push_back("Max_vel");
        rd_sdo.push_back("Max_tor");
        
        slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    return true;
}

bool Motor_ROS::get_motors_gains_info(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
   _status_callback=true;
    
    std::vector<std::string> slaves_selected;
    std::vector<std::string> rd_sdo;
    std::map<std::string ,std::string> wr_sdo;
    
    std::string cmd="get_motors_gains_info";
    
    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        rd_sdo.push_back("gain_0");
        rd_sdo.push_back("gain_1");
        rd_sdo.push_back("gain_2");
        rd_sdo.push_back("gain_3");
        rd_sdo.push_back("gain_4");
        
        slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
        
    }
    return true;
}


bool Motor_ROS::stop_motors(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    
    _status_callback=true;
    std::vector<std::string> slaves_selected;

    std::string cmd="stop_motors";
    
    if(_xbotcorepresence)
    {
        if((req.slave_name[0]!="all")&&(req.slave_name[0]!=""))
        {
            bool slave_found=false;
            for(int i=0; i<req.slave_name.size() && !slave_found;i++)
            {
                if(_motor_map_setup.count(req.slave_name[i])>0)
                {
                    slave_found=true;
                }
            }
            if(slave_found)
            {
                req.slave_name.clear();
                req.slave_name.push_back("all");
            }
        }
    }

    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        ctrl_cmd(slaves_selected,
                 _motor_map_setup,
                 _ec_client_cmd,
                 cmd,
                 iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                 res.slaves_info,
                 res.cmd_status,
                 _status_callback);
        
    }
    return true;
    
}


bool Motor_ROS::start_motors_posmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="start_motors_posmode";

        if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
        {
            for(int i=0; i<slaves_selected.size();i++)
            {
                if(_motor_map_setup.count(slaves_selected[i]))
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);
                    motor_setup->set_control_mode("3B_motor_pos_ctrl");
                }
            }
            
            ctrl_cmd(slaves_selected,
            _motor_map_setup,
            _ec_client_cmd,
            cmd,
            iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
            res.slaves_info,
            res.cmd_status,
            _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::start_motors_velmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
   if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="start_motors_velmode";

        if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
        {
            for(int i=0; i<slaves_selected.size();i++)
            {
                if(_motor_map_setup.count(slaves_selected[i]))
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);
                    motor_setup->set_control_mode("71_motor_vel_ctrl");
                }
            }
            
            ctrl_cmd(slaves_selected,
            _motor_map_setup,
            _ec_client_cmd,
            cmd,
            iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
            res.slaves_info,
            res.cmd_status,
            _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
    
};

bool Motor_ROS::start_motors_impmode(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="start_motors_impmode";

        if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
        {
            for(int i=0; i<slaves_selected.size();i++)
            {
                if(_motor_map_setup.count(slaves_selected[i]))
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);
                    motor_setup->set_control_mode("D4_impedance_ctrl");
                }
            }
            
            ctrl_cmd(slaves_selected,
            _motor_map_setup,
            _ec_client_cmd,
            cmd,
            iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
            res.slaves_info,
            res.cmd_status,
            _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::release_motors_brake(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    std::vector<std::string> slaves_selected;

    std::string cmd="release_motors_brake";
    
    if(_xbotcorepresence)
    {
        if((req.slave_name[0]!="all")&&(req.slave_name[0]!=""))
        {
            bool slave_found=false;
            for(int i=0; i<req.slave_name.size() && !slave_found;i++)
            {
                if(_motor_map_setup.count(req.slave_name[i])>0)
                {
                    slave_found=true;
                }
            }
            if(slave_found)
            {
                req.slave_name.clear();
                req.slave_name.push_back("all");
            }
        }
    }

    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        PDOs_aux_cmd(slaves_selected,
                    _motor_map_setup,
                    _ec_client_cmd,
                    cmd,
                    iit::advr::PDOs_aux_cmd_Aux_cmd_Type::PDOs_aux_cmd_Aux_cmd_Type_BRAKE_RELEASE,
                    res.slaves_info,
                    res.cmd_status,
                    _status_callback);
        
    }
    return true;
}

bool Motor_ROS::engage_motors_brake(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    _status_callback=true;
    std::vector<std::string> slaves_selected;

    std::string cmd="engage_motors_brake";
    
    if(_xbotcorepresence)
    {
        if((req.slave_name[0]!="all")&&(req.slave_name[0]!=""))
        {
            bool slave_found=false;
            for(int i=0; i<req.slave_name.size() && !slave_found;i++)
            {
                if(_motor_map_setup.count(req.slave_name[i])>0)
                {
                    slave_found=true;
                }
            }
            if(slave_found)
            {
                req.slave_name.clear();
                req.slave_name.push_back("all");
            }
        }
    }

    if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
    {
        PDOs_aux_cmd(slaves_selected,
                    _motor_map_setup,
                    _ec_client_cmd,
                    cmd,
                    iit::advr::PDOs_aux_cmd_Aux_cmd_Type::PDOs_aux_cmd_Aux_cmd_Type_BRAKE_ENGAGE,
                    res.slaves_info,
                    res.cmd_status,
                    _status_callback);
        
    }
    return true;
}

bool Motor_ROS::set_motors_posgains(ec_srvs::SetMotorPosVelGains::Request  &req,ec_srvs::SetMotorPosVelGains::Response &res)
{
    
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            motor_setup->set_control_mode("3B_motor_pos_ctrl");
            vector<float> pos_gains;
            for(int i=0;i<3;i++)
                    pos_gains.push_back(req.gains[i]);
            motor_setup->set_position_gains(pos_gains);
            motor_setup->set_data_object_type(req.data_object_type);
        }
        
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        std::vector<std::string> rd_sdo;
        std::map<std::string ,std::string> wr_sdo;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motors_posgains";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            if(req.data_object_type=="pdo")
            {
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS,
                res.slaves_info,
                res.cmd_status,
                _status_callback);
            }
            else
            {
                wr_sdo["gain_0"]=std::to_string(req.gains[0]);
                wr_sdo["gain_1"]=std::to_string(req.gains[1]);
                wr_sdo["gain_4"]=std::to_string(req.gains[2]);
            
                slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
            }
            
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::set_motors_velgains(ec_srvs::SetMotorPosVelGains::Request  &req,ec_srvs::SetMotorPosVelGains::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            motor_setup->set_control_mode("71_motor_vel_ctrl");
            vector<float> vel_gains;
            for(int i=0;i<3;i++)
                    vel_gains.push_back(req.gains[i]);

            motor_setup->set_velocity_gains(vel_gains);
            motor_setup->set_data_object_type(req.data_object_type);
        }
        
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        std::vector<std::string> rd_sdo;
        std::map<std::string ,std::string> wr_sdo;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motors_velgains";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            if(req.data_object_type=="pdo")
            {
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS,
                res.slaves_info,
                res.cmd_status,
                _status_callback);
            }
            else
            {
                wr_sdo["gain_0"]=std::to_string(req.gains[0]);
                wr_sdo["gain_1"]=std::to_string(req.gains[1]);
                wr_sdo["gain_4"]=std::to_string(req.gains[2]);
            
                slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
            }
            
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::set_motors_impgains(ec_srvs::SetMotorImpGains::Request  &req,ec_srvs::SetMotorImpGains::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         

        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            motor_setup->set_control_mode("D4_impedance_ctrl");
            vector<float> imp_gains;
            for(int i=0;i<5;i++)
                    imp_gains.push_back(req.gains[i]);
            motor_setup->set_impedance_gains(imp_gains);
            motor_setup->set_data_object_type(req.data_object_type);
        }
        
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        std::vector<std::string> rd_sdo;
        std::map<std::string ,std::string> wr_sdo;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motors_impgains";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            if(req.data_object_type=="pdo")
            {
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_GAINS,
                res.slaves_info,
                res.cmd_status,
                _status_callback);
            }
            else
            {
                wr_sdo["gain_0"]=std::to_string(req.gains[0]);
                wr_sdo["gain_1"]=std::to_string(req.gains[1]);
                wr_sdo["gain_2"]=std::to_string(req.gains[2]);
                wr_sdo["gain_3"]=std::to_string(req.gains[3]);
                wr_sdo["gain_4"]=std::to_string(req.gains[4]);
            
                slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
            }
            
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::set_motors_max_ampere(ec_srvs::SetSlaveCurrent::Request  &req,ec_srvs::SetSlaveCurrent::Response &res)
{
    
    if(!_xbotcorepresence)
    {
        _status_callback=true;
        
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        std::vector<std::string> rd_sdo;
        std::map<std::string ,std::string> wr_sdo;
        
        std::string cmd="set_motors_limits";
        
        for(int i=0; i < req.slave_name.size();i++)
        {
            slaves_requested.push_back(req.slave_name[i]);
            
            if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
            {
                wr_sdo["Max_curr"]=std::to_string(req.current[i]);
                
                slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
            }
            slaves_requested.pop_back();
            wr_sdo.clear();
        }
            

        return true;
    }
    else
    {
        return false;
    }
};



bool Motor_ROS::set_motors_limits(ec_srvs::SetMotorLimits::Request  &req,ec_srvs::SetMotorLimits::Response &res)
{
    
    if(!_xbotcorepresence)
    {
        _status_callback=true;
        
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        std::vector<std::string> rd_sdo;
        std::map<std::string ,std::string> wr_sdo;
        
        std::string cmd="set_motors_limits";
        
        slaves_requested.push_back(req.motor_name);
        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            wr_sdo["Min_pos"]=std::to_string(req.min_pos);
            wr_sdo["Max_pos"]=std::to_string(req.max_pos);
            wr_sdo["Max_vel"]=std::to_string(req.max_vel);
            wr_sdo["Max_tor"]=std::to_string(req.max_torq);
            
            slave_sdo_cmd(slaves_selected,_id_map_motors,_ec_client_cmd,cmd,rd_sdo,wr_sdo,res.slaves_info,res.cmd_status,_status_callback);
        }
            

        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::set_motors_position(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
        
        std::vector<std::string> slaves_selected;

        std::string cmd="set_motors_position";

        if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
        {
            std::vector<float> motors_position;
            if(slaves_request_adpating(req.motor_name,slaves_selected,req.position,motors_position,res.cmd_status))
            {
                for(int i=0; i<slaves_selected.size();i++)
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                    motor_setup->set_actual_position(motors_position[i]);
                }
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_POSITION,
                res.slaves_info,
                res.cmd_status,
                _status_callback);   
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
};
bool Motor_ROS::set_motors_velocity(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
    
        std::vector<std::string> slaves_selected;

        std::string cmd="set_motors_velocity";

        if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
        {
            std::vector<float> motors_velocity;
            if(slaves_request_adpating(req.motor_name,slaves_selected,req.velocity,motors_velocity,res.cmd_status))
            {
                for(int i=0; i<slaves_selected.size();i++)
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                    motor_setup->set_actual_velocity(motors_velocity[i]);
                }
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_VELOCITY,
                res.slaves_info,
                res.cmd_status,
                _status_callback);   
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
};
bool Motor_ROS::set_motors_torque(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="set_motors_torque";

        if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
        {
            std::vector<float> motors_torque;
            if(slaves_request_adpating(req.motor_name,slaves_selected,req.torque,motors_torque,res.cmd_status))
            {
                for(int i=0; i<slaves_selected.size();i++)
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                    motor_setup->set_actual_torque(motors_torque[i]);
                }
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_TORQUE,
                res.slaves_info,
                res.cmd_status,
                _status_callback);   
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
};
bool Motor_ROS::set_motors_amperage(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
        
        std::vector<std::string> slaves_selected;

        std::string cmd="set_motors_amperage";
    
        if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
        {
            std::vector<float> motors_amperage;
            if(slaves_request_adpating(req.motor_name,slaves_selected,req.amperage,motors_amperage,res.cmd_status))
            {
                for(int i=0; i<slaves_selected.size();i++)
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                    motor_setup->set_actual_amperage(motors_amperage[i]);
                }
                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_CURRENT,
                res.slaves_info,
                res.cmd_status,
                _status_callback); 
            }
            else
            {
                return false;
            }  
        }
        return true;
    }
    else
    {
        return false;
    }
};

bool Motor_ROS::set_motors_homing_position(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="set_motors_homing_position";

        if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
        {
            std::vector<float> motors_homing_position;
            if(slaves_request_adpating(req.motor_name,slaves_selected,req.homing_position,motors_homing_position,res.cmd_status))
            {
                for(int i=0; i<slaves_selected.size();i++)
                {
                    Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                    motor_setup->set_homing_position(motors_homing_position[i]);
                }

                ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_SET_HOME,
                res.slaves_info,
                res.cmd_status,
                _status_callback);   
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }  
}


bool Motor_ROS::set_motors_fan(ec_srvs::SetMotorCtrlValue::Request& req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    _status_callback = true;

    std::vector<std::string> slaves_selected;

    std::string cmd = "set_motors_fan";

    if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
    {
        std::vector<bool> motors_fan; 
        if(slaves_request_adpating(req.motor_name,slaves_selected,req.fan,motors_fan,res.cmd_status))
        {
            for(int i=0; i<slaves_selected.size();i++)
            {
                Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                motor_setup->set_fan_onoff(motors_fan[i]);
            }
            ctrl_cmd(slaves_selected,
                _motor_map_setup,
                _ec_client_cmd,
                cmd,
                iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_FAN,
                res.slaves_info,
                res.cmd_status,
                _status_callback);   
        }
        else
        {
            return false;
        } 
    }

    return true;
};

bool Motor_ROS::set_motors_led(ec_srvs::SetMotorCtrlValue::Request  &req,ec_srvs::SetMotorCtrlValue::Response &res)
{
    _status_callback=true;

    std::vector<std::string> slaves_selected;

    std::string cmd="set_motors_led";

    if(select_slaves(_internal_list_motors,req.motor_name,slaves_selected,res.cmd_status))
    {
        std::vector<bool> motors_led; 
        if(slaves_request_adpating(req.motor_name,slaves_selected,req.led,motors_led,res.cmd_status))
        {
            for(int i=0; i<slaves_selected.size();i++)
            {
                Motor_Setup::Ptr motor_setup=_motor_map_setup.at(slaves_selected[i]);  
                motor_setup->set_led_onoff(motors_led[i]);
            }
                        
            ctrl_cmd(slaves_selected,
            _motor_map_setup,
            _ec_client_cmd,
            cmd,
            iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_LED,
            res.slaves_info,
            res.cmd_status,
            _status_callback);  
        }
        else
        {
            return false;
        } 
    }
    return true;
};

bool Motor_ROS::set_motor_homing_trajectory(ec_srvs::SetMotorHomingTrj::Request  &req,ec_srvs::SetMotorHomingTrj::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            Motor_Setup::trajectoy_t trajectoy_s;
            trajectoy_s.trj_name=req.trajectory_name;
            trajectoy_s.homing_x.resize(req.x_home.size());
            trajectoy_s.homing_x=req.x_home;
            
            motor_setup->set_trajectory_structure(trajectoy_s);
        }
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motor_homing_trajectory";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            trajectory_cmd(slaves_selected,
                           _motor_map_setup,
                           _ec_client_cmd,
                           cmd,
                           iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_HOMING,
                           res.slaves_info,
                           res.cmd_status,
                           _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool Motor_ROS::set_motor_period_trajectory(ec_srvs::SetMotorPeriodTrj::Request  &req,ec_srvs::SetMotorPeriodTrj::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            Motor_Setup::trajectoy_t trajectoy_s;
            trajectoy_s.trj_name=req.trajectory_name;
            trajectoy_s.period_freq=req.frequency;
            trajectoy_s.period_ampl=req.amplitude;
            trajectoy_s.period_teta=req.theta;
            trajectoy_s.period_secs=req.secs;
        

            motor_setup->set_trajectory_structure(trajectoy_s);
        }
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motor_period_trajectory";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            trajectory_cmd(slaves_selected,
                           _motor_map_setup,
                           _ec_client_cmd,
                           cmd,
                           iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_SINE,
                           res.slaves_info,
                           res.cmd_status,
                           _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool Motor_ROS::set_motor_smooth_trajectory(ec_srvs::SetMotorSmoothTrj::Request  &req,ec_srvs::SetMotorSmoothTrj::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        if(_motor_map_setup.count(req.motor_name)>0)
        {
            Motor_Setup::Ptr motor_setup=_motor_map_setup.at(req.motor_name);
            Motor_Setup::trajectoy_t trajectoy_s;
            trajectoy_s.trj_name=req.trajectory_name;
            trajectoy_s.smooth_x.resize(req.x_smooth.size());
            trajectoy_s.smooth_x=req.x_smooth;

            trajectoy_s.smooth_y.resize(req.y_smooth.size());
            trajectoy_s.smooth_y=req.y_smooth;

            motor_setup->set_trajectory_structure(trajectoy_s);
        }
         
        std::vector<std::string> slaves_requested;
        std::vector<std::string> slaves_selected;
        
        slaves_requested.push_back(req.motor_name);

        std::string cmd="set_motor_smooth_trajectory";

        if(select_slaves(_internal_list_motors,slaves_requested,slaves_selected,res.cmd_status))
        {
            trajectory_cmd(slaves_selected,
                           _motor_map_setup,
                           _ec_client_cmd,
                           cmd,
                           iit::advr::Trajectory_cmd_Type::Trajectory_cmd_Type_SMOOTHER,
                           res.slaves_info,
                           res.cmd_status,
                           _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool Motor_ROS::start_motors_trajectory(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="start_motors_trajectory";

        if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
        {
            trj_queue_cmd(slaves_selected,
                          _motor_map_setup,
                          _ec_client_cmd,
                          cmd,
                          iit::advr::Trj_queue_cmd_Type::Trj_queue_cmd_Type_PUSH_QUE,
                          res.slaves_info,
                          res.cmd_status,
                          _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
    
}
bool Motor_ROS::clear_motors_trajectory(ec_srvs::SelectSlave::Request  &req,ec_srvs::SelectSlave::Response &res)
{
    if(!_xbotcorepresence)
    {
         _status_callback=true;
         
        std::vector<std::string> slaves_selected;

        std::string cmd="clear_motors_trajectory";

        if(select_slaves(_internal_list_motors,req.slave_name,slaves_selected,res.cmd_status))
        {
            trj_queue_cmd(slaves_selected,
                          _motor_map_setup,
                          _ec_client_cmd,
                          cmd,
                          iit::advr::Trj_queue_cmd_Type::Trj_queue_cmd_Type_EMPTY_QUE,
                          res.slaves_info,
                          res.cmd_status,
                          _status_callback);
            
        }
        return true;
    }
    else
    {
        return false;
    }
    
}
