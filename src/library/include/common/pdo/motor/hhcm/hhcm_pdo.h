#ifndef __HHCM_PDO__
#define __HHCM_PDO__

#include "common/pdo/motor/motor_pdo.h"

template <class T>
class HhcmPdo: public MotorPdo<T>{

public:

    HhcmPdo(const std::string ,int32_t id);
    HhcmPdo(const std::string ,int32_t id, uint32_t type);
    ~HhcmPdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
};

template < class T >
inline HhcmPdo<T>::HhcmPdo(std::string value,int id):
                           MotorPdo<T>(value,id)
{

};

template < class T >
inline HhcmPdo<T>::HhcmPdo(std::string value,int32_t id, uint32_t type):
                           MotorPdo<T>(value,id,type)
{
    
};

template < class T >
inline HhcmPdo<T>::~HhcmPdo()
{

};


template < class T >
inline void HhcmPdo<T>::get_from_pb() 
{
    std::get<0>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
    std::get<1>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
    std::get<2>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
    std::get<3>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
    std::get<4>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
    std::get<5>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
    std::get<6>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();
    std::get<7>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
    std::get<8>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt();
    std::get<9>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack();
    std::get<10>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux();
    std::get<11>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts();
    
    MotorPdo<T>::read_pos_ref           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
    MotorPdo<T>::read_vel_ref           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
    MotorPdo<T>::read_torque_ref        = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
}

template < class T >
inline void HhcmPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    //
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref(MotorPdo<T>::tx_pdo.pos_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_vel_ref(MotorPdo<T>::tx_pdo.vel_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_tor_ref(MotorPdo<T>::tx_pdo.tor_ref);

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0(MotorPdo<T>::tx_pdo.gain_0);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1(MotorPdo<T>::tx_pdo.gain_1);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2(MotorPdo<T>::tx_pdo.gain_2);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3(MotorPdo<T>::tx_pdo.gain_3);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4(MotorPdo<T>::tx_pdo.gain_4);
    
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(MotorPdo<T>::tx_pdo.op_idx_aux);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_aux(MotorPdo<T>::tx_pdo.aux);
}
#endif
