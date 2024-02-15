#ifndef __IMU_PDO__
#define __IMU_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/imu_Vn_esc.h>

template <class T>
class ImuPdo: public T{

public:

    ImuPdo(const std::string,int id);
    ~ImuPdo();
    
    void get_from_pb();

    void set_to_pb();

    iit::ecat::ImuEscPdoTypes::pdo_rx rx_pdo;
    std::vector<float> imu_v={0,0,0,0,0,0,0,0,0,0};
};

template < class T >
inline ImuPdo<T>::ImuPdo(std::string value,int id):
                       T(id,"Imu",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline ImuPdo<T>::~ImuPdo()
{
    T::write_quit();
};


template < class T >
inline void ImuPdo<T>::get_from_pb() 
{
    rx_pdo.x_rate       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate();
    imu_v[0]            = rx_pdo.x_rate;
    rx_pdo.y_rate       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate();
    imu_v[1]            = rx_pdo.y_rate;
    rx_pdo.z_rate       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate();
    imu_v[2]            = rx_pdo.y_rate;
    
    rx_pdo.x_acc        = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc();
    imu_v[3]            = rx_pdo.x_acc;
    rx_pdo.y_acc        = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc();
    imu_v[4]            = rx_pdo.y_acc;
    rx_pdo.z_acc        = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc();
    imu_v[5]            = rx_pdo.z_acc;
    
    if(T::pb_rx_pdos.mutable_imuvn_rx_pdo()->has_x_quat()){
        rx_pdo.x_quat       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat();
        imu_v[6]            = rx_pdo.x_quat;
        rx_pdo.y_quat       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat();
        imu_v[7]            = rx_pdo.y_quat;
        rx_pdo.z_quat       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat();
        imu_v[8]            = rx_pdo.z_quat;
        rx_pdo.w_quat       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat();
        imu_v[9]            = rx_pdo.w_quat;
    }

    rx_pdo.imu_ts       = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->imu_ts();
    rx_pdo.temperature  = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->temperature();
    rx_pdo.digital_in   = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->digital_in();
    rx_pdo.fault        = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->fault();
    rx_pdo.rtt          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->rtt();
}

template < class T >
inline void ImuPdo<T>::set_to_pb() 
{
}

template class ImuPdo<EcPipePdo>;
template class ImuPdo<EcZmqPdo>;



#endif
