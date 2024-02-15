#ifndef __PUMP_PDO__
#define __PUMP_PDO__

#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/hyq_hpu_esc.h>

template <class T>
class PumpPdo: public T{

public:

    PumpPdo(const std::string,int id);
    ~PumpPdo();
    
    void get_from_pb();

    void set_to_pb();
    
   iit::ecat::HyQ_HpuEscPdoTypes::pdo_rx rx_pdo;
   iit::ecat::HyQ_HpuEscPdoTypes::pdo_tx tx_pdo;

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
    rx_pdo.pressure               = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.tempExpansionBoard     = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.statusWord             = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1BoardTemp         = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1MotTemp           = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2BoardTemp         = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2MotTemp           = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1SwVer             = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1ActCur            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1ActSpd            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1Status            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    
    rx_pdo.vesc2SwVer             = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2ActCur            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2ActSpd            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2Status            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1ActSpd            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.temp1                  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.temp2                  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.temp3                  = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    
    rx_pdo.reserved               = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1FBDutyCycle       = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2FBDutyCycle       = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc1Demand            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.vesc2Demand            = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.pwm1DutyCycle          = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();
    rx_pdo.pwm2DutyCycle          = 0; //T::pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt();

}

template < class T >
inline void PumpPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    //T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.eCATInFrameCounter
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.demandPressure
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.singlePumpHighLt
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.singlePumpLowLt
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.HPUDemandMode
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.vesc1Mode
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.vesc2Mode
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.fan1Spd
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.fan2Spd
    //T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref();   tx_pdo.sysStateCmd

}

template class PumpPdo<EcPipePdo>;
template class PumpPdo<EcZmqPdo>;

#endif
