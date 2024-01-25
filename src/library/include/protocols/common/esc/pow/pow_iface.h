#ifndef __POW_IFACE__
#define __POW_IFACE__

#include <pb_utils.h>
#include "protocols/common/pipe/ec_pipe_pdo.h"
#include "protocols/common/zmq/ec_zmq_pdo.h"

typedef struct POW_PDO_t{
    
    std::vector<std::string>pow_pb_name = {"v_batt", "v_load", "i_load","temp_batt", "temp_heatsink", "temp_pcb"};
    std::vector<float> pow_v={0,0,0,0,0,0};
    
    // rx_pdo values
    float v_batt, v_load, i_load;
    float temp_batt, temp_heatsink, temp_pcb;
    float fault, status;
    
}POW_PDO_t;

template <class T>
class PowPdo: public T{

public:

    PowPdo(const std::string,int id);
    ~PowPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    POW_PDO_t _pow_pdo;

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
    _pow_pdo.v_batt              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    _pow_pdo.pow_v[0]            = _pow_pdo.v_batt;
    _pow_pdo.v_load              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
    _pow_pdo.pow_v[1]            = _pow_pdo.v_load;
    _pow_pdo.i_load              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
    _pow_pdo.pow_v[2]            = _pow_pdo.i_load;
    _pow_pdo.temp_batt           = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
    _pow_pdo.pow_v[3]            = _pow_pdo.temp_batt;
    _pow_pdo.temp_heatsink       = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
    _pow_pdo.pow_v[4]            = _pow_pdo.temp_heatsink;
    _pow_pdo.temp_pcb            = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
    _pow_pdo.pow_v[5]            = _pow_pdo.temp_pcb;
            
    _pow_pdo.status              = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
    _pow_pdo.fault               = T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
}

template < class T >
inline void PowPdo<T>::set_to_pb() 
{
}

template class PowPdo<EcPipePdo>;
template class PowPdo<EcZmqPdo>;

#endif
