#ifndef __PUMP_PDO__
#define __PUMP_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace PumpPdoRx{
    // not used tempExpansionBoard, vesc1SwVer, vesc2SwVer
    static const std::vector<std::string>name = {"pressure", "statusWord","vesc1BoardTemp", "vesc1MotTemp","vesc2BoardTemp",
                                                 "vesc2MotTemp","vesc1ActCur","vesc1ActSpd","vesc1Status",
                                                 "vesc2ActCur","vesc2ActSpd","vesc2Status","temp1","temp2","temp3",
                                                 "vesc1FBDutyCycle","vesc2FBDutyCycle","vesc1Demand","vesc2Demand","pwm1DutyCycle","pwm2DutyCycle"};
    static const int pdo_size=21;
    using pdo_t=std::tuple<uint8_t,uint16_t,uint8_t,uint8_t,uint8_t,uint8_t,float,
                           uint16_t,uint16_t,float,uint16_t,uint16_t,uint8_t,uint8_t,uint8_t,
                           uint32_t,uint32_t,float,float,uint32_t,uint32_t>;
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
        pdo_vector[15]= static_cast<T>(std::get<15>(pdo_tuple));
        pdo_vector[16]= static_cast<T>(std::get<16>(pdo_tuple));
        pdo_vector[17]= static_cast<T>(std::get<17>(pdo_tuple));
        pdo_vector[18]= static_cast<T>(std::get<18>(pdo_tuple));
        pdo_vector[19]= static_cast<T>(std::get<19>(pdo_tuple));
        pdo_vector[20]= static_cast<T>(std::get<20>(pdo_tuple));
        return true;
    }
};

namespace PumpPdoTx{
    static const std::vector<std::string>name = {"demandPressure", "singlePumpHighLt","singlePumpLowLt", "HPUDemandMode",
                                                 "vesc1Mode","vesc2Mode","fan1Spd","fan2Spd","sysStateCmd"};
    static const int pdo_size=9;
    using pdo_t=std::tuple<uint8_t, uint8_t, uint8_t, uint16_t, uint8_t,uint8_t,uint8_t,uint8_t,uint8_t>;
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

    PumpPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    PumpPdoTx::pdo_t tx_pdo={0,0,0,0,0,0,0,0,0};
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
    std::get<0>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->pressure();
    std::get<1>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->statusword();
    std::get<2>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1boardtemp();
    std::get<3>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1mottemp();
    std::get<4>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2boardtemp();
    std::get<5>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2mottemp();
    std::get<6>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1actcur();
    std::get<7>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1actspd();
    std::get<8>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1status();
    std::get<9>(rx_pdo)  = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2actcur();
    std::get<10>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2actspd();
    std::get<11>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2status();
    std::get<12>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->temp1();
    std::get<13>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->temp2();
    std::get<14>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->temp3();
    std::get<15>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1fbdutycycle();
    std::get<16>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2fbdutycycle();
    std::get<17>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc1demand();
    std::get<18>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->vesc2demand();
    std::get<19>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->pwm1dutycycle();
    std::get<20>(rx_pdo) = T::pb_rx_pdos.mutable_hyqhpu_rx_pdo()->pwm2dutycycle();

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
    
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_demandpressure(std::get<0>(tx_pdo));   
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_singlepumphighlt(std::get<1>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_singlepumplowlt(std::get<2>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_hpudemandmode(std::get<3>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_vesc1mode(std::get<4>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_vesc2mode(std::get<5>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_fan1spd(std::get<5>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_fan2spd(std::get<7>(tx_pdo));
    T::pb_tx_pdos.mutable_hyqhpu_tx_pdo()->set_sysstatecmd(std::get<8>(tx_pdo));
}

template class PumpPdo<EcPipePdo>;
template class PumpPdo<EcZmqPdo>;

#endif
