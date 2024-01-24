#ifndef __FT_IFACE__
#define __FT_IFACE__

#include <pb_utils.h>
#include "protocols/common/pipe/ec_pipe_pdo.h"

class ft6_iface: public EcPipePdo {

public:

    ft6_iface(std::string robot_name,
              int id);

    void get_from_pb(void);

    void set_to_pb(void);

    std::vector<std::string>ft_pb_name = {"force_x", "force_y", "force_z","torque_x", "torque_y", "torque_z"};
    std::vector<float> ft_v;
    
    // rx_pdo values
    
    float force_x, force_y, force_z;
    
    float torque_x, torque_y, torque_z;
    
    
    float aux;
    
    uint32_t op_idx_ack, fault;

};

inline ft6_iface::ft6_iface(std::string robot_name,
                     int id) :
        EcPipePdo(id,"Ft",robot_name)
{
    init();
    write_connect();
}


inline void ft6_iface::get_from_pb(void) 
{
    force_x          = pb_rx_pdos.mutable_ft6_rx_pdo()->force_x();
    force_y          = pb_rx_pdos.mutable_ft6_rx_pdo()->force_y();
    force_z          = pb_rx_pdos.mutable_ft6_rx_pdo()->force_z();
    
    torque_x         = pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x();
    torque_y         = pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y();
    torque_z         = pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z(); 
    
    aux              = pb_rx_pdos.mutable_ft6_rx_pdo()->aux();
    op_idx_ack       = pb_rx_pdos.mutable_ft6_rx_pdo()->op_idx_ack();
    fault            = pb_rx_pdos.mutable_ft6_rx_pdo()->fault();
}

inline void ft6_iface::set_to_pb(void) 
{
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);

}


#endif
