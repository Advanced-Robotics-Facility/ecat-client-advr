#ifndef __VALVE_PDO__
#define __VALVE_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"

typedef struct VALVE_PDO_t{
    
    std::vector<std::string>valve_pb_name = {"encoder_position", "torque", "pressure1","pressure2", "temperature"};
    std::vector<float> valve_v={0,0,0,0,0};
    
    // rx_pdo values
    float encoder_position,torque;           
    float pressure1,pressure2,temperature;   
    uint16_t fault;
    uint16_t rtt;               
    uint16_t op_idx_ack;
    float aux_rx;           
    
    // tx_pdo values
    float current_ref,aux_tx;
    uint16_t PWM1,PWM2,D_out,fault_ack,ts,op_idx_aux;
}VALVE_PDO_t;

template <class T>
class ValvePdo: public T{

public:

    ValvePdo(const std::string,int id);
    ~ValvePdo();
    
    void get_from_pb();

    void set_to_pb();
    
    VALVE_PDO_t _valve_pdo;

};

template < class T >
inline ValvePdo<T>::ValvePdo(std::string value,int id):
                            T(id,"PowBoard",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline ValvePdo<T>::~ValvePdo()
{
    T::write_quit();
};


template < class T >
inline void ValvePdo<T>::get_from_pb() 
{
    _valve_pdo.encoder_position    = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    _valve_pdo.valve_v[0]          = _valve_pdo.encoder_position;
    _valve_pdo.torque              = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
    _valve_pdo.valve_v[1]          = _valve_pdo.torque;
    _valve_pdo.pressure1           = 0;//T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
    _valve_pdo.valve_v[2]          = _valve_pdo.pressure1;
    _valve_pdo.pressure2           = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
    _valve_pdo.valve_v[3]          = _valve_pdo.pressure1;
    _valve_pdo.temperature         = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
    _valve_pdo.valve_v[4]          = _valve_pdo.temperature; 
    
    _valve_pdo.fault               = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
    _valve_pdo.op_idx_ack          = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
    _valve_pdo.aux_rx              = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
}

template < class T >
inline void ValvePdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    //T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.current_ref
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.aux_tx
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.PWM1
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.PWM2
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.D_out
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.fault_ack
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.ts
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   _valve_pdo.op_idx_aux
}

template class ValvePdo<EcPipePdo>;
template class ValvePdo<EcZmqPdo>;

#endif
