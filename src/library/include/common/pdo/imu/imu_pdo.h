#ifndef __IMU_PDO__
#define __IMU_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"

namespace ImuPdoRx{
    static const std::vector<std::string>name = {"x_rate", "y_rate", "z_rate","x_acc", "y_acc","z_acc",
                                                 "x_quat","y_quat","z_quat","w_quat","imu_ts","temperature","digital_in","fault","rtt"};
    static const int pdo_size=15;
    using pdo_t=std::tuple<float, float, float, float, float,float,float,float,float,float,uint32_t,uint16_t,uint16_t,uint16_t,uint16_t>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t pdo_tuple,std::vector<T>& pdo_vector){
        if(pdo_vector.size()!=pdo_size){
           return false;
        }
        pdo_vector[0]= std::get<0>(pdo_tuple);
        pdo_vector[1]= std::get<1>(pdo_tuple);
        pdo_vector[2]= std::get<2>(pdo_tuple);
        pdo_vector[3]= std::get<3>(pdo_tuple);
        pdo_vector[4]= std::get<4>(pdo_tuple);
        pdo_vector[5]= std::get<5>(pdo_tuple);
        pdo_vector[6]= std::get<6>(pdo_tuple);
        pdo_vector[7]= std::get<7>(pdo_tuple);
        pdo_vector[8]= std::get<8>(pdo_tuple);
        pdo_vector[9]= std::get<7>(pdo_tuple);
        pdo_vector[10]= std::get<10>(pdo_tuple);
        pdo_vector[11]= std::get<11>(pdo_tuple);
        pdo_vector[12]= std::get<12>(pdo_tuple);
        pdo_vector[13]= std::get<13>(pdo_tuple);
        pdo_vector[14]= std::get<13>(pdo_tuple);
        return true;
    }
};


template <class T>
class ImuPdo: public T{

public:

    ImuPdo(const std::string,int id);
    ~ImuPdo();
    
    void get_from_pb();

    void set_to_pb();

    ImuPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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
    std::get<0>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate();
    std::get<1>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate();
    std::get<2>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate();
    
    std::get<3>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc();
    std::get<4>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc();
    std::get<5>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc();
    
    if(T::pb_rx_pdos.mutable_imuvn_rx_pdo()->has_x_quat()){
        std::get<6>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat();
        std::get<7>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat();
        std::get<8>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat();
        std::get<9>(rx_pdo) = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat();
    }

    std::get<10>(rx_pdo)    = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->imu_ts();
    std::get<11>(rx_pdo)    = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->temperature();
    std::get<12>(rx_pdo)    = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->digital_in();
    std::get<13>(rx_pdo)    = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->fault();
    std::get<14>(rx_pdo)    = T::pb_rx_pdos.mutable_imuvn_rx_pdo()->rtt();
}

template < class T >
inline void ImuPdo<T>::set_to_pb() 
{
}

template class ImuPdo<EcPipePdo>;
template class ImuPdo<EcZmqPdo>;



#endif
