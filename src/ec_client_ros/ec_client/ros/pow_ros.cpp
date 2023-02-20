#include "pow_ros.h"

using namespace std;

POW_ROS::POW_ROS(std::shared_ptr<ros::NodeHandle> node):
_node(node)
{
    
    _pub_pow_pdo=_node->advertise<ec_msgs::PowPDO>("powPDO", 1);
}


std::map<std::string,EC_Client_PDO::Ptr> POW_ROS::get_ec_client_pdo_map()
{
    return _ec_client_pdo_map;
}

void POW_ROS::set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr>ec_client_pdo_map)
{
    if(!_ec_client_pdo_map.empty())
        _ec_client_pdo_map.clear();

    _ec_client_pdo_map=ec_client_pdo_map;
}

void POW_ROS::clear_pow_pdo_message()
{
    _pow_pdo.slave_name.clear();
    _pow_pdo.slave_name.resize(_internal_list_pows.size());
        
    _pow_pdo.v_batt.clear();
    _pow_pdo.v_batt.resize(_internal_list_pows.size());
    _pow_pdo.v_load.clear();
    _pow_pdo.v_load.resize(_internal_list_pows.size());
    
    _pow_pdo.i_load.clear();
    _pow_pdo.i_load.resize(_internal_list_pows.size());
    
    _pow_pdo.temp_batt.clear();
    _pow_pdo.temp_batt.resize(_internal_list_pows.size());
    _pow_pdo.temp_heatsink.clear();
    _pow_pdo.temp_heatsink.resize(_internal_list_pows.size());
    _pow_pdo.temp_pcb.clear();
    _pow_pdo.temp_pcb.resize(_internal_list_pows.size());

    _pow_pdo.status.clear();
    _pow_pdo.status.resize(_internal_list_pows.size());
    

    _pow_pdo.fault.clear();
    _pow_pdo.fault.resize(_internal_list_pows.size());
    
    _pow_pdo.fault_numb.clear();
    _pow_pdo.fault_numb.resize(_internal_list_pows.size());

}


void POW_ROS::set_internal_list_pows(std::vector<std::string> internal_list_pows)
{
    _internal_list_pows=internal_list_pows;
}

std::string POW_ROS::handle_fault(uint32_t fault)
{
    
    string fault_msg;
    
    return fault_msg;
                    
}

bool POW_ROS::pub_pow_pdo()
{
    for(int i=0;i<_internal_list_pows.size();i++)
    {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        string msg="";

        if(_ec_client_pdo_map.count(_internal_list_pows[i])>0)
        {
            EC_Client_PDO::Ptr ec_client_pdo=_ec_client_pdo_map.at(_internal_list_pows[i]);
            if(ec_client_pdo->zmq_cmd_recv_pdo(msg,pb_rx_pdos))
            {
                if(msg!="")
                {
                    _pow_pdo.slave_name[i]=_internal_list_pows[i];
                    
                    _pow_pdo.v_batt[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
                    _pow_pdo.v_load[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
                    
                    _pow_pdo.i_load[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
                    
                    _pow_pdo.temp_batt[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
                    _pow_pdo.temp_heatsink[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
                    _pow_pdo.temp_pcb[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
                    
                                        
                    _pow_pdo.status[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
                    
                    string fault_msg=handle_fault(pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault());
                    _pow_pdo.fault[i]=fault_msg;
                    _pow_pdo.fault_numb[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
                }
            }
            else
            {
                std::cout << "Error!" << std::endl;
                return false;
            }
        
        }
    }
    
    _pub_pow_pdo.publish(_pow_pdo);
    
    return true;
};
