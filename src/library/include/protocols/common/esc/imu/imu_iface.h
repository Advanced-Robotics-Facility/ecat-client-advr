#ifndef __IMU_IFACE__
#define __IMU_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"

class imu_iface : public esc_pipe_iface
{

public:

    imu_iface(std::string robot_name,
              int id);

    void get_from_pb(void);

    void set_to_pb(void);

    // rx_pdo values
    float x_rate, y_rate, z_rate;
    float x_acc, y_acc, z_acc;
    float x_quat, y_quat, z_quat, w_quat;
    uint32_t imu_ts, temperature, digital_in, fault;


};

inline imu_iface::imu_iface(std::string robot_name,
                     int id) :
    esc_pipe_iface(id,"Imu",robot_name)
{
}

inline void imu_iface::get_from_pb(void) 
{
    x_rate          = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate();
    y_rate          = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate();
    z_rate          = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate();
    
    x_acc           = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc();
    y_acc           = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc();
    z_acc           = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc();
    
    x_quat          = pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat();
    y_quat          = pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat();
    z_quat          = pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat();
    w_quat          = pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat();

    imu_ts          = pb_rx_pdos.mutable_imuvn_rx_pdo()->imu_ts();
    temperature     = pb_rx_pdos.mutable_imuvn_rx_pdo()->temperature();
    digital_in      = pb_rx_pdos.mutable_imuvn_rx_pdo()->digital_in();
    fault           = pb_rx_pdos.mutable_imuvn_rx_pdo()->fault();
}

inline void imu_iface::set_to_pb(void) 
{
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_IMU_VN);
}




#endif
