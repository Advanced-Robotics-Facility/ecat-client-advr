#ifndef __FT_PDO__
#define __FT_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
namespace FtPdoRx{
    static const std::vector<std::string>name = {"force_x", "force_y", "force_z","torque_x", "torque_y","torque_z","fault","rtt"};
    static const int pdo_size=8;
    using pdo_t=std::tuple<float, float, float, float, float,float,uint16_t,uint16_t>;
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
        return true;
    }
};

template <class T>
class FtPdo: public T{

public:

    FtPdo(const std::string,int id);
    ~FtPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    FtPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0};

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
    std::get<0>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_x();
    std::get<1>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_y();
    std::get<2>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->force_z();
    std::get<3>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x();
    std::get<4>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y();
    std::get<5>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z(); 
    std::get<6>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->fault();
    std::get<7>(rx_pdo)= T::pb_rx_pdos.mutable_ft6_rx_pdo()->rtt();
}

template < class T >
inline void FtPdo<T>::set_to_pb() 
{
}

template class FtPdo<EcPipePdo>;
template class FtPdo<EcZmqPdo>;


#endif
