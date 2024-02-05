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
    MotorPdo<T>::_motor_pdo.link_pos            = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
    std::get<0>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.link_pos;
    MotorPdo<T>::_motor_pdo.motor_pos           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
    std::get<1>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_pos;
    MotorPdo<T>::_motor_pdo.link_vel            = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
    std::get<2>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.link_vel;
    MotorPdo<T>::_motor_pdo.motor_vel           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
    std::get<3>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_vel;
    MotorPdo<T>::_motor_pdo.torque              = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
    std::get<4>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.torque;
    MotorPdo<T>::_motor_pdo.motor_temperature   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
    std::get<5>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_temperature;
    MotorPdo<T>::_motor_pdo.board_temperature   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();
    std::get<6>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.board_temperature;
    MotorPdo<T>::_motor_pdo.fault               = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
    std::get<7>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.fault;
    MotorPdo<T>::_motor_pdo.rtt                 = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt();
    std::get<8>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.rtt;
    MotorPdo<T>::_motor_pdo.aux_rd_idx_ack      = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack();
    std::get<9>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.aux_rd_idx_ack;
    MotorPdo<T>::_motor_pdo.aux_rd              = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux();
    std::get<10>(MotorPdo<T>::_motor_pdo.mt_t)  = MotorPdo<T>::_motor_pdo.aux_rd ;
    
    MotorPdo<T>::_motor_pdo.read_pos_ref        = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
    MotorPdo<T>::_motor_pdo.read_vel_ref        = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
    MotorPdo<T>::_motor_pdo.read_torque_ref     = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
    
    MotorPdo<T>::_motor_pdo.cmd_aux_sts         = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts();
    std::get<11>(MotorPdo<T>::_motor_pdo.mt_t)  = MotorPdo<T>::_motor_pdo.cmd_aux_sts ;
}

template < class T >
inline void HhcmPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    //
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref(MotorPdo<T>::_motor_pdo.pos_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_vel_ref(MotorPdo<T>::_motor_pdo.vel_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_tor_ref(MotorPdo<T>::_motor_pdo.tor_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0(MotorPdo<T>::_motor_pdo.kp_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1(MotorPdo<T>::_motor_pdo.kd_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2(MotorPdo<T>::_motor_pdo.tau_p_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3(MotorPdo<T>::_motor_pdo.tau_d_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4(MotorPdo<T>::_motor_pdo.tau_fc_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(MotorPdo<T>::_motor_pdo.aux_rd_idx_req);
}
#endif
