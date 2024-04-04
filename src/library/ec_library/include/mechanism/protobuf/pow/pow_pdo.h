#ifndef __POW_PDO__
#define __POW_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace PowPdoRx{
    static const std::vector<std::string>name = {"v_batt", "v_load", "i_load","temp_batt", "temp_heatsink","temp_pcb","status","fault","op_idx_ack","aux"};
    static const int pdo_size=10;
    using pdo_t=std::tuple<float, float, float, float, float,float,uint16_t,uint16_t,uint16_t,float>;
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
        pdo_vector[9]= std::get<9>(pdo_tuple);
        return true;
    }
};


template <class T>
class PowPdo: public T{

public:

    PowPdo(const std::string,int id);
    ~PowPdo();
    
    void get_from_pb();

    void set_to_pb();
    
    PowPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0};
    bool init_rx_pdo=false;
};

template < class T >
inline PowPdo<T>::PowPdo(std::string value,int id):
                       T(id,"PowBoard",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline PowPdo<T>::~PowPdo()
{
    T::write_quit();
};

template < class T >
inline void PowPdo<T>::get_from_pb() 
{
    std::get<0>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<1>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load();
    std::get<2>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load();
    std::get<3>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt();
    std::get<4>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink();
    std::get<5>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb();
    std::get<6>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->status();
    std::get<7>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->fault();
    std::get<8>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->op_idx_ack();
    std::get<9>(rx_pdo)= T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->aux();

    if(!init_rx_pdo){
        init_rx_pdo=true;   
    }
}

template < class T >
inline void PowPdo<T>::set_to_pb() 
{
}

template class PowPdo<EcPipePdo>;
template class PowPdo<EcZmqPdo>;

#endif
