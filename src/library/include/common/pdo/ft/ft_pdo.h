#ifndef __FT_PDO__
#define __FT_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"


typedef struct FT_PDO_t{
    std::vector<std::string>ft_pb_name = {"force_x", "force_y", "force_z","torque_x", "torque_y", "torque_z"};
    std::vector<float> ft_v={0,0,0,0,0,0};
    
    // rx_pdo values
    
    float force_x, force_y, force_z;
    
    float torque_x, torque_y, torque_z;
    
    
    float aux;
    
    uint32_t op_idx_ack, fault;
    
}FT_PDO;

template <class T>
class FtPdo: public T{

public:

    FtPdo(const std::string,int id);
    ~FtPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    FT_PDO _ft_pdo;

};

template < class T >
inline FtPdo<T>::FtPdo(std::string value,int id):
                     T(id,"Ft",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline FtPdo<T>::~FtPdo()
{
    T::write_quit();
};

template < class T >
inline void FtPdo<T>::get_from_pb() 
{
    _ft_pdo.force_x          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_x();
    _ft_pdo.ft_v[0]          = _ft_pdo.force_x;
    _ft_pdo.force_y          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_y();
    _ft_pdo.ft_v[1]          = _ft_pdo.force_y;
    _ft_pdo.force_z          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_z();
    _ft_pdo.ft_v[2]          = _ft_pdo.force_z;
    
    _ft_pdo.torque_x         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x();
    _ft_pdo.ft_v[3]          = _ft_pdo.torque_x;
    _ft_pdo.torque_y         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y();
    _ft_pdo.ft_v[4]          = _ft_pdo.torque_y;
    _ft_pdo.torque_z         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z(); 
    _ft_pdo.ft_v[5]          = _ft_pdo.torque_z;
    
    _ft_pdo.aux              = T::pb_rx_pdos.mutable_ft6_rx_pdo()->aux();
    _ft_pdo.op_idx_ack       = T::pb_rx_pdos.mutable_ft6_rx_pdo()->op_idx_ack();
    _ft_pdo.fault            = T::pb_rx_pdos.mutable_ft6_rx_pdo()->fault();
}

template < class T >
inline void FtPdo<T>::set_to_pb() 
{
}

template class FtPdo<EcPipePdo>;
template class FtPdo<EcZmqPdo>;


#endif
