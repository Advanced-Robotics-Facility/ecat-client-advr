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
    std::get<0>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->link_pos();
    std::get<1>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->motor_pos();
    std::get<2>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->link_vel();
    std::get<3>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->motor_vel();
    std::get<4>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->torque(); 
    std::get<5>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->drive_temp();
    std::get<6>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->drive_temp();
    std::get<7>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->statusword();
    std::get<8>(MotorPdo<T>::rx_pdo)    = 0; //rtt
    std::get<9>(MotorPdo<T>::rx_pdo)    = 0; //op_idx_ack
    std::get<10>(MotorPdo<T>::rx_pdo)   = 0; //aux
    std::get<11>(MotorPdo<T>::rx_pdo)   = 0; //cmd_aux_sts
    
    MotorPdo<T>::read_pos_ref           = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_pos();
    MotorPdo<T>::read_vel_ref           = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_vel();
    MotorPdo<T>::read_torque_ref        = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_torque();
    MotorPdo<T>::read_curr_ref          = T::pb_rx_pdos.mutable_circulo9_rx_pdo()->demanded_current();
}

template < class T >
inline void Circulo9Pdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_CIRCULO9);

    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_pos(std::get<1>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_vel(std::get<2>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_torque(std::get<3>(MotorPdo<T>::tx_pdo));  
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_target_current(MotorPdo<T>::curr_ref);
    
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_0(std::get<4>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_1(std::get<5>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_2(std::get<6>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_3(std::get<7>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_circulo9_tx_pdo()->set_gain_4(std::get<8>(MotorPdo<T>::tx_pdo));
}
#endif
