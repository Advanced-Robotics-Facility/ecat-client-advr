#ifndef __ADVRF_PDO__
#define __ADVRF_PDO__

#include "mechanism/protobuf/motor/motor_pdo.h"

template <class T>
class AdvrfPdo: public MotorPdo<T>{

public:

    AdvrfPdo(const std::string ,int32_t id);
    AdvrfPdo(const std::string ,int32_t id, uint32_t type);
    ~AdvrfPdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
};

template < class T >
inline AdvrfPdo<T>::AdvrfPdo(std::string value,int id):
                           MotorPdo<T>(value,id)
{
    MotorPdo<T>::init_pb();
};

template < class T >
inline AdvrfPdo<T>::AdvrfPdo(std::string value,int32_t id, uint32_t type):
                           MotorPdo<T>(value,id,type)
{
    MotorPdo<T>::init_pb();
};

template < class T >
inline AdvrfPdo<T>::~AdvrfPdo()
{

};


template < class T >
inline void AdvrfPdo<T>::get_from_pb() 
{
    std::get<0>(MotorPdo<T>::rx_pdo)    = 0; // status word NOT YET implemented!!!!
    std::get<1>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
    std::get<2>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
    std::get<3>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
    std::get<4>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
    std::get<5>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
    std::get<6>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux();
    std::get<7>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
    std::get<8>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();
    std::get<9>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
    std::get<10>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt();
    std::get<11>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
    std::get<12>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
    std::get<13>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
    if(MotorPdo<T>::_ctrl_type_cast == iit::advr::Gains_Type_CURRENT){
        std::get<13>(MotorPdo<T>::rx_pdo)   = 0;
        std::get<14>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
    }
    
    if(!MotorPdo<T>::init_rx_pdo){
        MotorPdo<T>::init_rx_pdo=true;   
    }
}

template < class T >
inline void AdvrfPdo<T>::set_to_pb() 
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
    
    MotorPdo<T>::_ctrl_type_cast = static_cast<iit::advr::Gains_Type>(std::get<0>(MotorPdo<T>::tx_pdo));

    if((MotorPdo<T>::_ctrl_type_cast == iit::advr::Gains_Type_POSITION ||
        MotorPdo<T>::_ctrl_type_cast == iit::advr::Gains_Type_VELOCITY)) {
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
