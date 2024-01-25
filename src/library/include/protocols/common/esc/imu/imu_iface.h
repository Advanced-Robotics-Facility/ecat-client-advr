#ifndef __IMU_IFACE__
#define __IMU_IFACE__

#include <pb_utils.h>
#include "protocols/common/pipe/ec_pipe_pdo.h"
#include "protocols/common/zmq/ec_zmq_pdo.h"


typedef struct IMU_PDO_t{
    
    std::vector<std::string>imu_pb_name = {"x_rate", "y_rate", "z_rate","x_acc", "y_acc", "z_acc","x_quat","y_quat","z_quat","w_quat"};
    std::vector<float> imu_v={0,0,0,0,0,0,0,0,0,0};
    
    // rx_pdo values
    float x_rate, y_rate, z_rate;
    float x_acc, y_acc, z_acc;
    float x_quat, y_quat, z_quat, w_quat;
    uint32_t imu_ts, temperature, digital_in, fault;

}IMU_PDO;

template <class T>
class ImuPdo: public T{

public:

    ImuPdo(const std::string,int id);
    ~ImuPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    IMU_PDO _imu_pdo;

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
    _imu_pdo.x_rate          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate();
    _imu_pdo.imu_v[0]        = _imu_pdo.x_rate;
    _imu_pdo.y_rate          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate();
    _imu_pdo.imu_v[1]        = _imu_pdo.y_rate;
    _imu_pdo.z_rate          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate();
    _imu_pdo.imu_v[2]        = _imu_pdo.y_rate;
    
    _imu_pdo.x_acc           = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc();
    _imu_pdo.imu_v[3]        = _imu_pdo.x_acc;
    _imu_pdo.y_acc           = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc();
    _imu_pdo.imu_v[4]        = _imu_pdo.y_acc;
    _imu_pdo.z_acc           = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc();
    _imu_pdo.imu_v[5]        = _imu_pdo.z_acc;
    
    _imu_pdo.x_quat          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat();
    _imu_pdo.imu_v[6]        = _imu_pdo.x_quat;
    _imu_pdo.y_quat          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat();
    _imu_pdo.imu_v[7]        = _imu_pdo.y_quat;
    _imu_pdo.z_quat          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat();
    _imu_pdo.imu_v[8]        = _imu_pdo.z_quat;
    _imu_pdo.w_quat          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat();
    _imu_pdo.imu_v[9]        = _imu_pdo.w_quat;

    _imu_pdo.imu_ts          = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->imu_ts();
    _imu_pdo.temperature     = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->temperature();
    _imu_pdo.digital_in      = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->digital_in();
    _imu_pdo.fault           = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->fault();
}

template < class T >
inline void ImuPdo<T>::set_to_pb() 
{
}

template class ImuPdo<EcPipePdo>;
template class ImuPdo<EcZmqPdo>;



#endif
