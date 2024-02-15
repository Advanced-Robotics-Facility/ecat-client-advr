#ifndef __FT_PDO__
#define __FT_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/ft6_esc.h>

template <class T>
class FtPdo: public T{

public:

    FtPdo(const std::string,int id);
    ~FtPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    iit::ecat::Ft6EscPdoTypes::pdo_rx rx_pdo;
    std::vector<float> ft_v={0,0,0,0,0,0};

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
    rx_pdo.force_X          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_x();
    ft_v[0]                 = rx_pdo.force_X;
    rx_pdo.force_Y          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_y();
    ft_v[1]                 = rx_pdo.force_Y;
    rx_pdo.force_Z          = T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_z();
    ft_v[2]                 = rx_pdo.force_Z;
    
    rx_pdo.torque_X         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x();
    ft_v[3]                 = rx_pdo.torque_X;
    rx_pdo.torque_Y         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y();
    ft_v[4]                 = rx_pdo.torque_Y;
    rx_pdo.torque_Z         = T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z(); 
    ft_v[5]                 = rx_pdo.torque_Z;
    
    rx_pdo.rtt              = T::pb_rx_pdos.mutable_ft6_rx_pdo()->rtt();
    rx_pdo.fault            = T::pb_rx_pdos.mutable_ft6_rx_pdo()->fault();
}

template < class T >
inline void FtPdo<T>::set_to_pb() 
{
}

template class FtPdo<EcPipePdo>;
template class FtPdo<EcZmqPdo>;


#endif
