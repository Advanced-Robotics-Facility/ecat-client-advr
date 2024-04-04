#ifndef __HHCM_PDO__
#define __HHCM_PDO__

#include "mechanism/protobuf/motor/motor_pdo.h"

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

    if(!MotorPdo<T>::init_rx_pdo){
        MotorPdo<T>::init_rx_pdo=true;   
    }
}

template < class T >
inline void HhcmPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    //
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref(std::get<1>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_vel_ref(std::get<2>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_tor_ref(std::get<3>(MotorPdo<T>::tx_pdo));  
    
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0(std::get<4>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1(std::get<5>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2(std::get<6>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3(std::get<7>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4(std::get<8>(MotorPdo<T>::tx_pdo));
    
    auto ctrl_type_cast = static_cast<iit::advr::Gains_Type>(std::get<0>(MotorPdo<T>::tx_pdo));

    if((ctrl_type_cast == iit::advr::Gains_Type_POSITION ||
        ctrl_type_cast == iit::advr::Gains_Type_VELOCITY)) {
        T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0(std::get<4>(MotorPdo<T>::tx_pdo));
        T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1(std::get<6>(MotorPdo<T>::tx_pdo));
        T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2(0.0);
        T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3(0.0);
        T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4(std::get<5>(MotorPdo<T>::tx_pdo));
    }

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);
                
    auto _op = static_cast<iit::advr::AuxPDO_Op>(std::get<9>(MotorPdo<T>::tx_pdo));

    switch (_op)
    {
        case iit::advr::AuxPDO_Op_SET:{
            T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(std::get<10>(MotorPdo<T>::tx_pdo));
            T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_aux(std::get<11>(MotorPdo<T>::tx_pdo));
        }break;
        case iit::advr::AuxPDO_Op_GET:
            T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(std::get<10>(MotorPdo<T>::tx_pdo));
            break;
        case iit::advr::AuxPDO_Op_NOP:
            break;
    }
}
#endif
