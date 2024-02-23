#ifndef __VALVE_PDO__
#define __VALVE_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/hyq_knee_esc.h>

typedef struct VALVE_PDO_t{
    
    std::vector<std::string>valve_pb_name = {"encoder_position", "torque", "pressure1","pressure2", "temperature"};
}VALVE_PDO_t;

template <class T>
class ValvePdo: public T{

public:

    ValvePdo(const std::string,int id);
    ~ValvePdo();
    
    void get_from_pb();

    void set_to_pb();
    
    iit::ecat::HyQ_KneeEscPdoTypes::pdo_rx rx_pdo;
    iit::ecat::HyQ_KneeEscPdoTypes::pdo_tx tx_pdo;
    
    std::vector<float> valve_v={0,0,0,0,0};

};

template < class T >
inline ValvePdo<T>::ValvePdo(std::string value,int id):
                            T(id,"HyQ_KneeESC",value)
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
    rx_pdo.encoder_position    = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->encoder_position();
    valve_v[0]                 = rx_pdo.encoder_position;
    rx_pdo.torque              = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->torque();
    valve_v[1]                 = rx_pdo.torque;
    rx_pdo.pressure_1          = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->pressure_1();
    valve_v[2]                 = rx_pdo.pressure_1;
    rx_pdo.pressure_2          = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->pressure_2();
    valve_v[3]                 = rx_pdo.pressure_1;
    rx_pdo.temperature         = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->temperature();
    valve_v[4]                 = rx_pdo.temperature; 
    
    rx_pdo.fault               = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->fault();
    rx_pdo.op_idx_ack          = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->op_idx_ack();
    rx_pdo.aux                 = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->aux();
    rx_pdo.rtt                 = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->rtt();
    
}

template < class T >
inline void ValvePdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_HYQ_KNEE);
    
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_current_ref(tx_pdo.current_ref);
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_pwm_1(tx_pdo.pwm_1);  
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_pwm_2(tx_pdo.pwm_2);
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_dout(tx_pdo.dout);
}

template class ValvePdo<EcPipePdo>;
template class ValvePdo<EcZmqPdo>;

#endif
