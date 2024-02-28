#ifndef __PUMP_PDO__
#define __PUMP_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"

namespace PumpPdoRx{
    static const std::vector<std::string>name = {"pressure", "tempExpansionBoard", "statusWord","vesc1BoardTemp", "vesc1MotTemp","vesc2BoardTemp",
                                                 "vesc2MotTemp","vesc1SwVer","vesc1ActCur","vesc1ActSpd","vesc1Status","vesc2SwVer",
                                                 "vesc2ActCur","vesc2ActSpd","vesc2Status","temp1","temp2","temp3",
                                                 "vesc1FBDutyCycle","vesc2FBDutyCycle","vesc1Demand","vesc2Demand","pwm1DutyCycle","pwm2DutyCycle"};
    static const int pdo_size=24;
    using pdo_t=std::tuple<uint8_t,uint8_t,uint16_t,uint8_t,uint8_t,uint8_t,uint8_t,float,float,
                           uint16_t,uint16_t,float,float,uint16_t,uint16_t,uint8_t,uint8_t,uint8_t,
                           uint32_t,uint32_t,float,float,uint32_t,uint32_t>;
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
        pdo_vector[10]= std::get<10>(pdo_tuple);
        pdo_vector[11]= std::get<11>(pdo_tuple);
        pdo_vector[12]= std::get<12>(pdo_tuple);
        pdo_vector[13]= std::get<13>(pdo_tuple);
        pdo_vector[14]= std::get<14>(pdo_tuple);
        pdo_vector[15]= std::get<15>(pdo_tuple);
        pdo_vector[16]= std::get<16>(pdo_tuple);
        pdo_vector[17]= std::get<17>(pdo_tuple);
        pdo_vector[18]= std::get<18>(pdo_tuple);
        pdo_vector[19]= std::get<19>(pdo_tuple);
        pdo_vector[20]= std::get<20>(pdo_tuple);
        pdo_vector[21]= std::get<21>(pdo_tuple);
        pdo_vector[22]= std::get<22>(pdo_tuple);
        pdo_vector[23]= std::get<23>(pdo_tuple);
        return true;
    }
};

namespace PumpPdoTx{
    static const std::vector<std::string>name = {"demandPressure", "singlePumpHighLt","singlePumpLowLt", "HPUDemandMode",
                                                 "vesc1Mode","vesc2Mode","fan1Spd","fan2Spd","sysStateCmd"};
    static const int pdo_size=9;
    using pdo_t=std::tuple<uint8_t, uint8_t, uint8_t, uint16_t, uint8_t,uint8_t,uint8_t,uint8_t,uint8_t>;
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
    
   PumpPdoRx::pdo_t  rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   PumpPdoTx::pdo_t tx_pdo={0,0,0,0,0,0,0,0,0};

};

template < class T >
inline PumpPdo<T>::PumpPdo(std::string value,int id):
                       T(id,"HyQ_HpuESC",value)
{
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
    std::get<0>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<1>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<2>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<3>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<4>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<5>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<6>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<7>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<8>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<9>(rx_pdo)  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<10>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<11>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<12>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<13>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<14>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<15>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<16>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<17>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<18>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<19>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<20>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<21>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<22>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    std::get<23>(rx_pdo) = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
}

template < class T >
inline void PumpPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    //T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<0>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<1>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<2>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<3>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<4>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<5>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<6>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<7>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<8>(tx_pdo)
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   std::get<9>(tx_pdo)

}

template class PumpPdo<EcPipePdo>;
template class PumpPdo<EcZmqPdo>;

#endif
