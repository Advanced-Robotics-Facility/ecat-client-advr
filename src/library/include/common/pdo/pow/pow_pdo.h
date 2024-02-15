#ifndef __POW_PDO__
#define __POW_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/power_f28m36_board.h>

template <class T>
class PowPdo: public T{

public:

    PowPdo(const std::string,int id);
    ~PowPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    iit::ecat::PowF28M36EscPdoTypes::pdo_rx rx_pdo;
    std::vector<float> pow_v={0,0,0,0,0,0};

};

template < class T >
inline PowPdo<T>::PowPdo(std::string value,int id):
                       T(id,"PowBoard",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline PowPdo<T>::~PowPdo()
{
    T::write_quit();
};

template < class T >
inline void PowPdo<T>::get_from_pb() 
{
    rx_pdo.v_batt              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    pow_v[0]                   = rx_pdo.v_batt;
    rx_pdo.v_load              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
    pow_v[1]                   = rx_pdo.v_load;
    rx_pdo.i_load              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
    pow_v[2]                   = rx_pdo.i_load;
    rx_pdo.temp_batt           = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
    pow_v[3]                   = rx_pdo.temp_batt;
    rx_pdo.temp_heatsink       = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
    pow_v[4]                   = rx_pdo.temp_heatsink;
    rx_pdo.temp_pcb            = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
    pow_v[5]                   = rx_pdo.temp_pcb;
            
    rx_pdo.status.all          = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
    rx_pdo.fault               = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
    rx_pdo.op_idx_ack          = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->op_idx_ack();
    rx_pdo.aux                 = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->aux();
}

template < class T >
inline void PowPdo<T>::set_to_pb() 
{
}

template class PowPdo<EcPipePdo>;
template class PowPdo<EcZmqPdo>;

#endif
