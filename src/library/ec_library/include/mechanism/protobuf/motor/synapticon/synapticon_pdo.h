#ifndef __SYNAPTICON_PDO__
#define __SYNAPTICON_PDO__

#include "mechanism/protobuf/motor/motor_pdo.h"

template <class T>
class SynapticonPdo: public MotorPdo<T>{

public:

    SynapticonPdo(const std::string ,int32_t id);
    SynapticonPdo(const std::string ,int32_t id, uint32_t type);
    ~SynapticonPdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
};

template < class T >
inline SynapticonPdo<T>::SynapticonPdo(std::string value,int id):
                                   MotorPdo<T>(value,id)
{
    MotorPdo<T>::init_pb();
};

template < class T >
inline SynapticonPdo<T>::SynapticonPdo(std::string value,int32_t id, uint32_t type):
                                   MotorPdo<T>(value,id,type)
{
    MotorPdo<T>::init_pb();
};

template < class T >
inline SynapticonPdo<T>::~SynapticonPdo()
{

};


template < class T >
inline void SynapticonPdo<T>::get_from_pb() 
{
    std::get<0>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->statusword();
    std::get<1>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->link_pos();
    std::get<2>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->motor_pos();
    std::get<3>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->link_vel();
    std::get<4>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->motor_vel();
    std::get<5>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->torque(); 
    std::get<6>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->current(); 
    std::get<7>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->motor_temp();
    std::get<8>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->drive_temp();
    std::get<9>(MotorPdo<T>::rx_pdo)    = T::pb_rx_pdos.mutable_cia402_rx_pdo()->error_code();
    std::get<10>(MotorPdo<T>::rx_pdo)   = 0;
    std::get<11>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->demanded_pos();
    std::get<12>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->demanded_vel();
    std::get<13>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->demanded_torque();
    std::get<14>(MotorPdo<T>::rx_pdo)   = T::pb_rx_pdos.mutable_cia402_rx_pdo()->demanded_current();

    if(!MotorPdo<T>::init_rx_pdo){
        MotorPdo<T>::init_rx_pdo=true;   
    }
}

template < class T >
inline void SynapticonPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_CIA402);

    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_pos(std::get<1>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_vel(std::get<2>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_torque(std::get<3>(MotorPdo<T>::tx_pdo));  
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_target_current(std::get<3>(MotorPdo<T>::tx_pdo));
    
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_gain_0(std::get<4>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_gain_1(std::get<5>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_gain_2(std::get<6>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_gain_3(std::get<7>(MotorPdo<T>::tx_pdo));
    T::pb_tx_pdos.mutable_cia402_tx_pdo()->set_gain_4(std::get<8>(MotorPdo<T>::tx_pdo));
}
#endif
