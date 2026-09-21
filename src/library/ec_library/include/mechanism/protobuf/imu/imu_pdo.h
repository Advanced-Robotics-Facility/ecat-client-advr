#ifndef __IMU_PDO__
#define __IMU_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace ImuPdoRx{
    static const std::vector<std::string>name = {"x_rate", "y_rate", "z_rate","x_acc", "y_acc","z_acc",
                                                 "x_quat","y_quat","z_quat","w_quat","imu_ts","temperature","digital_in","fault","rtt"};
    static const int pdo_size=15;
    using pdo_t=std::tuple<float, float, float, float, float,float,float,float,float,float,uint32_t,uint16_t,uint16_t,uint16_t,uint16_t>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t &pdo_tuple,std::vector<T> &pdo_vector){
        if(pdo_vector.size()!=pdo_size){
           return false;
        }
        pdo_vector[0]= static_cast<T>(std::get<0>(pdo_tuple));
        pdo_vector[1]= static_cast<T>(std::get<1>(pdo_tuple));
        pdo_vector[2]= static_cast<T>(std::get<2>(pdo_tuple));
        pdo_vector[3]= static_cast<T>(std::get<3>(pdo_tuple));
        pdo_vector[4]= static_cast<T>(std::get<4>(pdo_tuple));
        pdo_vector[5]= static_cast<T>(std::get<5>(pdo_tuple));
        pdo_vector[6]= static_cast<T>(std::get<6>(pdo_tuple));
        pdo_vector[7]= static_cast<T>(std::get<7>(pdo_tuple));
        pdo_vector[8]= static_cast<T>(std::get<8>(pdo_tuple));
        pdo_vector[9]= static_cast<T>(std::get<9>(pdo_tuple));
        pdo_vector[10]= static_cast<T>(std::get<10>(pdo_tuple));
        pdo_vector[11]= static_cast<T>(std::get<11>(pdo_tuple));
        pdo_vector[12]= static_cast<T>(std::get<12>(pdo_tuple));
        pdo_vector[13]= static_cast<T>(std::get<13>(pdo_tuple));
        pdo_vector[14]= static_cast<T>(std::get<14>(pdo_tuple));
        return true;
    }
};

namespace ImuPdoTx{
    static const std::vector<std::string>name = {"digital_out"};
    static const int pdo_size=1;
    using pdo_t=std::tuple<uint16_t>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t &pdo_tuple,std::vector<T> &pdo_vector){
        if(pdo_vector.size()!=pdo_size){
           return false;
        }
        pdo_vector[0]= static_cast<T>(std::get<0>(pdo_tuple));
        return true;
    }
};

template <class T>
class ImuPdo: public T{

public:

    ImuPdo(const std::string ,int32_t id, uint32_t type);
    ~ImuPdo();
    
    void get_from_pb();

    void set_to_pb();

    ImuPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ImuPdoTx::pdo_t tx_pdo={0};
    bool init_rx_pdo=false;
private:
    void init_pb();
};

template < class T >
inline void ImuPdo<T>::init_pb() 
{
   uint8_t  pb_buf[MAX_PB_SIZE];
   uint32_t msg_size=0;

   set_to_pb();
   msg_size = T::pb_tx_pdos.ByteSizeLong();
   T::pb_tx_pdos.SerializeToArray( (void*)(pb_buf+sizeof(msg_size)), msg_size);
}

template < class T >
inline ImuPdo<T>::ImuPdo(const std::string value,int32_t id, uint32_t type):
                       T(id, type, value)
{
    init_pb();
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

    if(!init_rx_pdo){
        init_rx_pdo=true;   
    }
}

template < class T >
inline void ImuPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_IMU_VN);
    T::pb_tx_pdos.mutable_imuvn_tx_pdo()->set_digital_out(std::get<0>(tx_pdo));
}

template class ImuPdo<EcPipePdo>;
template class ImuPdo<EcZmqPdo>;



#endif
