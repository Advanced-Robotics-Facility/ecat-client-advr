#include "imu_ros.h"

using namespace std;

IMU_ROS::IMU_ROS(std::shared_ptr<ros::NodeHandle> node):
_node(node)
{
    
    _pub_imu_pdo=_node->advertise<ec_msgs::ImuPDO>("imuPDO", 1);
}


std::map<std::string,EC_Client_PDO::Ptr> IMU_ROS::get_ec_client_pdo_map()
{
    return _ec_client_pdo_map;
}

void IMU_ROS::set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr>ec_client_pdo_map)
{
    if(!_ec_client_pdo_map.empty())
        _ec_client_pdo_map.clear();

    _ec_client_pdo_map=ec_client_pdo_map;
}

void IMU_ROS::clear_imu_pdo_message()
{
    _imu_pdo.slave_name.clear();
    _imu_pdo.slave_name.resize(_internal_list_imus.size());
        
    _imu_pdo.x_rate.clear();
    _imu_pdo.x_rate.resize(_internal_list_imus.size());
    _imu_pdo.y_rate.clear();
    _imu_pdo.y_rate.resize(_internal_list_imus.size());
    _imu_pdo.z_rate.clear();
    _imu_pdo.z_rate.resize(_internal_list_imus.size());
    
    _imu_pdo.x_acc.clear();
    _imu_pdo.x_acc.resize(_internal_list_imus.size());
    _imu_pdo.y_acc.clear();
    _imu_pdo.y_acc.resize(_internal_list_imus.size());
    _imu_pdo.z_acc.clear();
    _imu_pdo.z_acc.resize(_internal_list_imus.size());
    
    _imu_pdo.x_quat.clear();
    _imu_pdo.x_quat.resize(_internal_list_imus.size());
    _imu_pdo.y_quat.clear();
    _imu_pdo.y_quat.resize(_internal_list_imus.size());
    _imu_pdo.z_quat.clear();
    _imu_pdo.z_quat.resize(_internal_list_imus.size());
    _imu_pdo.w_quat.clear();
    _imu_pdo.w_quat.resize(_internal_list_imus.size());
    
    _imu_pdo.imu_ts.clear();
    _imu_pdo.imu_ts.resize(_internal_list_imus.size());
    
    _imu_pdo.temperature.clear();
    _imu_pdo.temperature.resize(_internal_list_imus.size());
    
    _imu_pdo.digital_in.clear();
    _imu_pdo.digital_in.resize(_internal_list_imus.size());

    _imu_pdo.fault.clear();
    _imu_pdo.fault.resize(_internal_list_imus.size());
    
    _imu_pdo.fault_numb.clear();
    _imu_pdo.fault_numb.resize(_internal_list_imus.size());
    
}


void IMU_ROS::set_internal_list_imus(std::vector<std::string> internal_list_imus)
{
    _internal_list_imus=internal_list_imus;
}

std::string IMU_ROS::handle_fault(uint32_t fault)
{
    
    string fault_msg;
    
    return fault_msg;
                    
}

bool IMU_ROS::pub_imu_pdo()
{
    for(int i=0;i<_internal_list_imus.size();i++)
    {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        string msg="";

        if(_ec_client_pdo_map.count(_internal_list_imus[i])>0)
        {
            EC_Client_PDO::Ptr ec_client_pdo=_ec_client_pdo_map.at(_internal_list_imus[i]);
            if(ec_client_pdo->zmq_cmd_recv_pdo(msg,pb_rx_pdos))
            {
                if(msg!="")
                {
                    _imu_pdo.slave_name[i]=_internal_list_imus[i];
                    
                    _imu_pdo.x_rate[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate();
                    _imu_pdo.y_rate[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate();
                    _imu_pdo.z_rate[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate();

                    _imu_pdo.x_acc[i]           = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc();
                    _imu_pdo.y_acc[i]           = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc();
                    _imu_pdo.z_acc[i]           = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc();

                    _imu_pdo.x_quat[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat();
                    _imu_pdo.y_quat[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat();
                    _imu_pdo.z_quat[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat();
                    _imu_pdo.w_quat[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat();

                    _imu_pdo.imu_ts[i]          = pb_rx_pdos.mutable_imuvn_rx_pdo()->imu_ts();
                    _imu_pdo.temperature[i]     = pb_rx_pdos.mutable_imuvn_rx_pdo()->temperature();
                    _imu_pdo.digital_in[i]      = pb_rx_pdos.mutable_imuvn_rx_pdo()->digital_in();
                    
                    string fault_msg=handle_fault(pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault());
                    _imu_pdo.fault[i]=fault_msg;
                    _imu_pdo.fault_numb[i]=pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
                }
            }
            else
            {
                std::cout << "Error!" << std::endl;
                return false;
            }
        
        }
    }
    
    _pub_imu_pdo.publish(_imu_pdo);
    
    return true;
};
