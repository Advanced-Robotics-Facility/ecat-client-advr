#ifndef __PUMP_PDO__
#define __PUMP_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace PumpPdoRx{
    // not used tempExpansionBoard, vesc1SwVer, vesc2SwVer
    static const std::vector<std::string>name = {"Motor_Current", "Motor_Speed","Pressure1", "Pressure2",
                                                 "temperature","MOSFET_temperature","motor_temperature",
                                                 "fault","rtt","op_idx_ack","aux"};
    static const int pdo_size=11;
    using pdo_t=std::tuple<float,float,float,float,
                           uint16_t,uint16_t,int16_t,
                           uint16_t,uint16_t,uint16_t,float>;
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
        return true;
    }
};

namespace PumpPdoTx{
    static const std::vector<std::string>name = {"Pump_Target",
                                                 "pressure_P_gain","pressure_I_gain", "pressure_D_gain","pressure_I_limit",
                                                 "fault_ack","SolenoidOut","ts","op_idx_aux","aux"};
    static const int pdo_size=10;
    using pdo_t=std::tuple<float, 
                           float, float, float, float,
                           uint16_t,uint16_t,uint16_t,uint16_t,float>;
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
        return true;
    }
};

template <class T>
class PumpPdo: public T{

public:

    PumpPdo(const std::string,int id);
    ~PumpPdo();

    void get_from_pb();

    void set_to_pb();

    PumpPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0};
    PumpPdoTx::pdo_t tx_pdo={0,0,0,0,0,0,0,0,0,0};
    bool init_rx_pdo=false;
private:
    void init_pb();
};

template < class T >
inline void PumpPdo<T>::init_pb() 
{
   uint8_t  pb_buf[MAX_PB_SIZE];
   uint32_t msg_size=0;

   set_to_pb();
   msg_size = T::pb_tx_pdos.ByteSizeLong();
   T::pb_tx_pdos.SerializeToArray( (void*)(pb_buf+sizeof(msg_size)), msg_size);
}

template < class T >
inline PumpPdo<T>::PumpPdo(std::string value,int id):
                       T(id,"HyQ_HpuESC",value)
{
    init_pb();
    T::init();
    T::write_connect();
};

template < class T >
inline PumpPdo<T>::~PumpPdo()
{
    T::write_quit();
};


template < class T >
inline void PumpPdo<T>::get_from_pb() 
{
    std::get<0>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->motor_current();
    std::get<1>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->motor_speed();
    std::get<2>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->pressure1();
    std::get<3>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->pressure2();
    std::get<4>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->temperature();
    std::get<5>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->mosfet_temperature();
    std::get<6>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->motor_temperature();
    std::get<7>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->fault();
    std::get<8>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->rtt();
    std::get<9>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->op_idx_ack();
    std::get<10>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->aux();
   
    if(!init_rx_pdo){
        init_rx_pdo=true;   
    }
}

template < class T >
inline void PumpPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_HYQ_HPU);
    
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_pump_target(std::get<0>(tx_pdo));   
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_pressure_p_gain(std::get<1>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_pressure_i_gain(std::get<2>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_pressure_d_gain(std::get<3>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_pressure_i_limit(std::get<4>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_fault_ack(std::get<5>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_solenoidout(std::get<6>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_ts(std::get<7>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_op_idx_aux(std::get<8>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_aux(std::get<9>(tx_pdo));
}

template class PumpPdo<EcPipePdo>;
template class PumpPdo<EcZmqPdo>;

#endif
