#ifndef __CIRCULO9_PDO__
#define __CIRCULO9_PDO__

#include "common/pdo/motor/motor_pdo.h"

template <class T>
class Circulo9Pdo: public MotorPdo<T>{

public:

    Circulo9Pdo(const std::string ,int32_t id);
    Circulo9Pdo(const std::string ,int32_t id, uint32_t type);
    ~Circulo9Pdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
};

template < class T >
inline Circulo9Pdo<T>::Circulo9Pdo(std::string value,int id):
                                   MotorPdo<T>(value,id)
{

};

template < class T >
inline Circulo9Pdo<T>::Circulo9Pdo(std::string value,int32_t id, uint32_t type):
                                   MotorPdo<T>(value,id,type)
{
    
};

template < class T >
inline Circulo9Pdo<T>::~Circulo9Pdo()
{

};


template < class T >
inline void Circulo9Pdo<T>::get_from_pb() 
{
    MotorPdo<T>::_motor_pdo.link_pos            = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->link_pos();
    std::get<0>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.link_pos;
    MotorPdo<T>::_motor_pdo.motor_pos           = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->motor_pos();
    std::get<1>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_pos;
    MotorPdo<T>::_motor_pdo.link_vel            = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->link_vel();
    std::get<2>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.link_vel;
    MotorPdo<T>::_motor_pdo.motor_vel           = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->motor_vel();
    std::get<3>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_vel;
    MotorPdo<T>::_motor_pdo.torque              = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->torque(); // _rx_iface.control_effort = pb_rx_pdos.mutable_circulo9_rx_pdo()->control_effort();
    std::get<4>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.torque;
    MotorPdo<T>::_motor_pdo.motor_temperature   = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->drive_temp();
    std::get<5>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.motor_temperature;
    MotorPdo<T>::_motor_pdo.board_temperature   = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->drive_temp();
    std::get<6>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.board_temperature;
    MotorPdo<T>::_motor_pdo.fault               = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->statusword();
    std::get<7>(MotorPdo<T>::_motor_pdo.mt_t)   = MotorPdo<T>::_motor_pdo.fault;
    
    //MotorPdo<T>::_motor_pdo.rtt                 = 0
    std::get<8>(MotorPdo<T>::_motor_pdo.mt_t)   = 0;
   
    //MotorPdo<T>::_motor_pdo.aux_rd_idx_ack      = 0;
    std::get<9>(MotorPdo<T>::_motor_pdo.mt_t)   = 0;
    
    //MotorPdo<T>::_motor_pdo.aux_rd              = 0;
    std::get<10>(MotorPdo<T>::_motor_pdo.mt_t)  = 0;
    
    MotorPdo<T>::_motor_pdo.read_pos_ref        = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_pos();
    MotorPdo<T>::_motor_pdo.read_vel_ref        = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_vel();
    MotorPdo<T>::_motor_pdo.read_torque_ref     = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_torque();
    MotorPdo<T>::_motor_pdo.read_curr_ref       = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_current();
    
    //MotorPdo<T>::_motor_pdo.cmd_aux_sts         = 0;
    std::get<11>(MotorPdo<T>::_motor_pdo.mt_t)  = 0;
}

template < class T >
inline void Circulo9Pdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_CIRCULO9);
    //
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_pos(MotorPdo<T>::_motor_pdo.pos_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_vel(MotorPdo<T>::_motor_pdo.vel_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_torque(MotorPdo<T>::_motor_pdo.tor_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_current(MotorPdo<T>::_motor_pdo.curr_ref);
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));

    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_0(MotorPdo<T>::_motor_pdo.kp_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_1(MotorPdo<T>::_motor_pdo.kd_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_2(MotorPdo<T>::_motor_pdo.tau_p_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_3(MotorPdo<T>::_motor_pdo.tau_d_ref);
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_4(MotorPdo<T>::_motor_pdo.tau_fc_ref);
    
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(MotorPdo<T>::_motor_pdo.aux_rd_idx_req);
}
#endif
