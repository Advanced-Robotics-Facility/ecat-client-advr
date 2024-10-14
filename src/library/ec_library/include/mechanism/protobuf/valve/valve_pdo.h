#ifndef __VALVE_PDO__
#define __VALVE_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace ValvePdoRx{
    static const std::vector<std::string>name = {"encoder_position", "force", "pressure1","pressure2", "current","temperature",
                                                 "fault","rtt","op_idx_ack","aux",
                                                 "current_ref_fb", "position_ref_fb", "force_ref_fb"};
    static const int pdo_size=13;
    using pdo_t=std::tuple<float, float, float, float, float, float,uint16_t,uint16_t,uint16_t,float,float,float,float>;
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
        return true;
    }
};

namespace ValvePdoTx{
    static const std::vector<std::string>name = {"current_ref", "position_ref", "force_ref",
                                                 "gain_0","gain_1","gain_2","gain_3","gain_4",
                                                 "fault_ack","ts","op_idx_aux","aux"};
    static const int pdo_size=12;
    using pdo_t=std::tuple<float, float, float,float,float,float,float,float,uint16_t, uint16_t,uint16_t,float>;
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
        return true;
    }
};

template <class T>
class ValvePdo: public T{

public:

    ValvePdo(const std::string,int id);
    ~ValvePdo();
    
    void get_from_pb();

    void set_to_pb();
    
    ValvePdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0,0};
    ValvePdoTx::pdo_t tx_pdo={0,0,0,0,0,0,0,0,0,0,0,0};
    bool init_rx_pdo=false;
private:
    void init_pb();
};

template < class T >
inline void ValvePdo<T>::init_pb() 
{
   uint8_t  pb_buf[MAX_PB_SIZE];
   uint32_t msg_size=0;

   set_to_pb();
   msg_size = T::pb_tx_pdos.ByteSizeLong();
   T::pb_tx_pdos.SerializeToArray( (void*)(pb_buf+sizeof(msg_size)), msg_size);
}

template < class T >
inline ValvePdo<T>::ValvePdo(std::string value,int id):
                            T(id,"HyQ_KneeESC",value)
{
    init_pb();
    T::init();
    T::write_connect();
};

template < class T >
inline ValvePdo<T>::~ValvePdo()
{
    T::write_quit();
};


template < class T >
inline void ValvePdo<T>::get_from_pb() 
{
    std::get<0>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->encoder_position();
    std::get<1>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->force();
    std::get<2>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->pressure_1();
    std::get<3>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->pressure_2();
    std::get<4>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->current();
    std::get<5>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->temperature();
    std::get<6>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->fault();
    std::get<7>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->rtt();
    std::get<8>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->op_idx_ack();
    std::get<9>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->aux();
    std::get<10>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->current_ref_fb();
    std::get<11>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->position_ref_fb();
    std::get<12>(rx_pdo)   = T::pb_rx_pdos.mutable_hyqknee_rx_pdo()->force_ref_fb();

    if(!init_rx_pdo){
        init_rx_pdo=true;
    }
}

template < class T >
inline void ValvePdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_HYQ_KNEE);
    
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_current_ref(std::get<0>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_position_ref(std::get<1>(tx_pdo));  
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_force_ref(std::get<2>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_gain_0(std::get<3>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_gain_1(std::get<4>(tx_pdo));  
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_gain_2(std::get<5>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_gain_3(std::get<6>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_gain_4(std::get<7>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_fault_ack(std::get<8>(tx_pdo));  
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_ts(std::get<9>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_op_idx_aux(std::get<10>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqknee_tx_pdo()->set_aux(std::get<11>(tx_pdo));
}

template class ValvePdo<EcPipePdo>;
template class ValvePdo<EcZmqPdo>;

#endif
