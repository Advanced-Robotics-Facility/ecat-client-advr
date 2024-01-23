#include "protocols/common/ec_pdo.h"


EcPdo::EcPdo(std::string protocol, std::string host_address, uint32_t host_port):
_protocol(protocol),
_host_address(host_address),
_host_port(host_port)
{
    if(_host_address=="localhost")
    {
        _host_address.clear();
        _host_address="127.0.0.1";
    }
    
}

EcPdo::~EcPdo()
{

}

void EcPdo::esc_factory(SSI slave_descr)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
    
            std::string host_port_cmd = std::to_string(_host_port+id);
            // zmq setup
            std::string zmq_uri = _protocol+"://" + _host_address + ":"+host_port_cmd;
            EcZmqPdo::Ptr zmq_pdo = std::make_shared<EcZmqPdo>(zmq_uri);
            
            switch ( esc_type  )
            {
                    case CENT_AC:
                    case LO_PWR_DC_MC :
                        {
                            _moto_pdo_map[id]=zmq_pdo;
                        }
                        break;
                    case FT6 :
                        {
                            _ft_pdo_map[id]=zmq_pdo;
                        }
                        break;
                        
                    case IMU_ANY :
                        {
                             _imu_pdo_map[id]=zmq_pdo;
                        }
                        break;
                        
                    case POW_F28M36_BOARD :
                        {
                            _pow_pdo_map[id]=zmq_pdo;
                        }
                        break;
                    
                    default:
                        break;
            }               
    }
} 

void EcPdo::read_motors(MotorStatusMap &motor_status_map)
{
    for ( auto &[id, moto_pdo] : _moto_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(moto_pdo->read())
        {
            if(msg!="")
            {
                motor_status_map[id] = std::make_tuple(pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts());
            }
        }
        else
        {
            std::cout << "Error on motor reading on device: " << id << std::endl;
        }
    }
}
void EcPdo::read_fts(FtStatusMap &ft_status_map)
{
    for ( auto &[id, ft_pdo] : _ft_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(ft_pdo->read())
        {
            if(msg!="")
            {
                std::vector<float> ft_v;
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_x());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_z());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_y());
                
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z());
                
                ft_status_map[id]=ft_v;
            }
        }
        else
        {
            std::cout << "Error on Force/Torque reading on device: "<< id << std::endl;
        }
    }
}
void EcPdo::read_imus(ImuStatusMap &imu_status_map)
{
    for ( auto &[id, imu_pdo] : _imu_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(imu_pdo->read())
        {
            if(msg!="")
            {
                std::vector<float> imu_v;
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate());
                
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc());
                
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat());
                imu_status_map[id]=imu_v;
            }
        }
        else
        {
           std::cout << "Error on IMU reading on device: " << id << std::endl;
        }
    }
}

void EcPdo::read_pows(PwrStatusMap &pow_status_map)
{
    for ( auto &[id, pow_pdo] : _pow_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(pow_pdo->read())
        {
            if(msg!="")
            {
                std::vector<float> pow_v;
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load());
                
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb());
                
                pow_status_map[id]=pow_v;
            }
        }
        else
        {
            std::cout << "Error on power board reading on device: " << id << std::endl;
        }
    }
}
