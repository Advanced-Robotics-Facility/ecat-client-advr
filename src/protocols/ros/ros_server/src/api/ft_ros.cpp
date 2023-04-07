#include "ft_ros.h"

using namespace std;

FT_ROS::FT_ROS(std::shared_ptr<ros::NodeHandle> node):
_node(node)
{
    
    _pub_ft_pdo=_node->advertise<ec_msgs::FtPDO>("ftPDO", 1);
}


std::map<std::string,EC_Client_PDO::Ptr> FT_ROS::get_ec_client_pdo_map()
{
    return _ec_client_pdo_map;
}

void FT_ROS::set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr>ec_client_pdo_map)
{
    if(!_ec_client_pdo_map.empty())
        _ec_client_pdo_map.clear();

    _ec_client_pdo_map=ec_client_pdo_map;
}

void FT_ROS::clear_ft_pdo_message()
{
    _ft_pdo.slave_name.clear();
    _ft_pdo.slave_name.resize(_internal_list_fts.size());
        
    _ft_pdo.force_x.clear();
    _ft_pdo.force_x.resize(_internal_list_fts.size());
    _ft_pdo.force_y.clear();
    _ft_pdo.force_y.resize(_internal_list_fts.size());
    _ft_pdo.force_z.clear();
    _ft_pdo.force_z.resize(_internal_list_fts.size());
    
    _ft_pdo.torque_x.clear();
    _ft_pdo.torque_x.resize(_internal_list_fts.size());
    _ft_pdo.torque_y.clear();
    _ft_pdo.torque_y.resize(_internal_list_fts.size());
    _ft_pdo.torque_z.clear();
    _ft_pdo.torque_z.resize(_internal_list_fts.size());

    _ft_pdo.aux.clear();
    _ft_pdo.aux.resize(_internal_list_fts.size());
    _ft_pdo.op_idx_ack.clear();
    _ft_pdo.op_idx_ack.resize(_internal_list_fts.size());
    

    _ft_pdo.fault.clear();
    _ft_pdo.fault.resize(_internal_list_fts.size());
    
    _ft_pdo.fault_numb.clear();
    _ft_pdo.fault_numb.resize(_internal_list_fts.size());

}


void FT_ROS::set_internal_list_fts(std::vector<std::string> internal_list_fts)
{
    _internal_list_fts=internal_list_fts;
}

std::string FT_ROS::handle_fault(uint32_t fault)
{
    
    string fault_msg;
    
    return fault_msg;
                    
}

bool FT_ROS::pub_ft_pdo()
{
    for(int i=0;i<_internal_list_fts.size();i++)
    {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        string msg="";

        if(_ec_client_pdo_map.count(_internal_list_fts[i])>0)
        {
            EC_Client_PDO::Ptr ec_client_pdo=_ec_client_pdo_map.at(_internal_list_fts[i]);
            if(ec_client_pdo->zmq_cmd_recv_pdo(msg,pb_rx_pdos))
            {
                if(msg!="")
                {
                    _ft_pdo.slave_name[i]=_internal_list_fts[i];
                    
                    _ft_pdo.force_x[i]          =pb_rx_pdos.mutable_ft6_rx_pdo()->force_x();
                    _ft_pdo.force_y[i]          =pb_rx_pdos.mutable_ft6_rx_pdo()->force_y();
                    _ft_pdo.force_z[i]          =pb_rx_pdos.mutable_ft6_rx_pdo()->force_z();
                    
                    
                    _ft_pdo.torque_x[i]         =pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x();
                    _ft_pdo.torque_y[i]         =pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y();
                    _ft_pdo.torque_z[i]         =pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z();
                    
                    _ft_pdo.aux[i]              = pb_rx_pdos.mutable_ft6_rx_pdo()->aux();
                    _ft_pdo.op_idx_ack[i]       = pb_rx_pdos.mutable_ft6_rx_pdo()->op_idx_ack();
                    
                    
                    string fault_msg=handle_fault(pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault());
                    _ft_pdo.fault[i]=fault_msg;
                    _ft_pdo.fault_numb[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
                }
            }
            else
            {
                std::cout << "Error!" << std::endl;
                return false;
            }
        
        }
    }
    
    _pub_ft_pdo.publish(_ft_pdo);
    
    return true;
};
