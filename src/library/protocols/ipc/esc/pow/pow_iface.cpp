#include "pow_iface.h"


powf28m36_iface::powf28m36_iface(std::string robot_name,
                                 int id) :
    esc_pipe_iface(id,"PowBoard",robot_name)
{
}

void powf28m36_iface::get_from_pb(void) 
{
    v_batt              = pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    v_load              = pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
    i_load              = pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
    temp_batt           = pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
    temp_heatsink       = pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
    temp_pcb            = pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
            
    status              = pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
    fault               = pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
}

void powf28m36_iface::set_to_pb(void) 
{
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    // Type
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_POW_F28M36);
}



