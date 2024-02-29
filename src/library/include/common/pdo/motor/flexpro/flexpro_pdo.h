#ifndef __FLEXPRO_PDO__
#define __FLEXPRO_PDO__

#include "common/pdo/motor/motor_pdo.h"

template <class T>
class FlexproPdo: public MotorPdo<T>{

public:

    FlexproPdo(const std::string ,int32_t id);
    FlexproPdo(const std::string ,int32_t id, uint32_t type);
    ~FlexproPdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
};

template < class T >
inline FlexproPdo<T>::FlexproPdo(std::string value,int id):
                                   MotorPdo<T>(value,id)
{

};

template < class T >
inline FlexproPdo<T>::FlexproPdo(std::string value,int32_t id, uint32_t type):
                                   MotorPdo<T>(value,id,type)
{
    
};

template < class T >
inline FlexproPdo<T>::~FlexproPdo()
{

};


template < class T >
inline void FlexproPdo<T>::get_from_pb() 
{
    std::get<0>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->actual_pos();
    std::get<1>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->actual_pos();
    std::get<2>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->actual_vel();
    std::get<3>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->actual_vel();
    std::get<4>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->actual_cur(); 
    std::get<5>(MotorPdo<T>::rx_pdo)    = 0;
    std::get<6>(MotorPdo<T>::rx_pdo)    = 0;
    std::get<7>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->status_word();
    std::get<8>(MotorPdo<T>::rx_pdo)    = 0; //rtt
    std::get<9>(MotorPdo<T>::rx_pdo)    = 0; //op_idx_ack
    std::get<10>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->drive_status();
    std::get<11>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->modes_of_op_display();
    
    MotorPdo<T>::read_pos_ref               = 0;
    MotorPdo<T>::read_vel_ref               = 0;
    MotorPdo<T>::read_torque_ref            = 0;
    MotorPdo<T>::read_curr_ref              = 0;
}

template < class T >
inline void FlexproPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_CIA402);
    //
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_control_word(0);
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_modes_of_op(0);
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_pos(MotorPdo<T>::tx_pdo.pos_ref);
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_vel(MotorPdo<T>::tx_pdo.vel_ref);
    //T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_torque(MotorPdo<T>::tx_pdo.tor_ref);
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_cur(MotorPdo<T>::curr_ref);
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));
    
//     T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_0(MotorPdo<T>::tx_pdo.gain_0);
//     T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_1(MotorPdo<T>::tx_pdo.gain_1);
//     T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_2(MotorPdo<T>::tx_pdo.gain_2);
//     T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_3(MotorPdo<T>::tx_pdo.gain_3);
//     T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_4(MotorPdo<T>::tx_pdo.gain_4);
//     
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(MotorPdo<T>::tx_pdo.aux_rd_idx_req);
}
#endif
