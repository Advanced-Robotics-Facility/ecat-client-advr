#include "ft_iface.h"

ft6_iface::ft6_iface(std::string robot_name,
                     int id) :
    esc_pipe_iface(id,"Ft",robot_name)
{
}

void ft6_iface::get_from_pb(void) 
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

void ft6_iface::set_to_pb(void) 
{
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);

}



