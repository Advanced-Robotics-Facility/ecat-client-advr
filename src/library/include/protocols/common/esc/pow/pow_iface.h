#ifndef __POW_IFACE__
#define __POW_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"

class powf28m36_iface: public esc_pipe_iface
{

public:

    powf28m36_iface(std::string robot_name,
                    int id);

    void get_from_pb(void);
    void set_to_pb(void);

    // rx_pdo values
    float v_batt, v_load, i_load;
    float temp_batt, temp_heatsink, temp_pcb;
    float fault, status;
    std::vector<float> pow_v;

};

inline powf28m36_iface::powf28m36_iface(std::string robot_name,
                                 int id) :
    esc_pipe_iface(id,"PowBoard",robot_name)
{
    init();
    write_connect();
}

inline void powf28m36_iface::get_from_pb(void) 
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

inline void powf28m36_iface::set_to_pb(void) 
{
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    // Type
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_POW_F28M36);
}




#endif
