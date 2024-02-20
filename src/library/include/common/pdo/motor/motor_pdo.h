#ifndef __MOTOR_PDO__
#define __MOTOR_PDO__

#include<tuple>
#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"
#include <esc/esc.h>


typedef struct MOTOR_PDO_t{
    std::vector<std::string>motor_pb_name = {"link_pos", "motor_pos", "link_vel",
                                             "motor_vel", "torque", "motor_temp",
                                             "board_temp","fault","rtt","op_idx_ack","aux","cmd_aux_sts"};
    
            
}MOTOR_PDO;

template <class T>
class MotorPdo: public T{

public:

    MotorPdo(const std::string ,int32_t id);
    MotorPdo(const std::string ,int32_t id, uint32_t type);
    ~MotorPdo();

    virtual void get_from_pb(void)=0;

    virtual void set_to_pb(void)=0;
    
    iit::ecat::McEscPdoTypes::pdo_rx rx_pdo;
    float read_pos_ref,read_vel_ref,read_torque_ref,read_curr_ref; // should be added in rx_pdo
    float motor_temperature,board_temperature; // should be added in rx_pdo
    uint32_t cmd_aux_sts; // should be added in rx_pdo

    iit::ecat::McEscPdoTypes::pdo_tx tx_pdo;
    float curr_ref; // should be added in tx_pdo
    
    std::tuple<float, float, float, float,   // pos_{link,motor}, vel_{link,motor}
            float,// torque
            float,float,                  // {motor,board}
            uint32_t, uint32_t,           // fault, rtt, op_idx_ack   // aux // cmd_aux_sts               
            uint32_t, float, uint32_t> mt_t={0,0,0,0,0,0,0,0,0,0,0,0};  

};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int id):
                           T(id,"Motor",value)
{
    tx_pdo.pos_ref = tx_pdo.vel_ref = tx_pdo.tor_ref = curr_ref= 0.0;
    
    T::init();
    T::write_connect();
};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int32_t id, uint32_t type):
                           T(id, type, value)
{
    tx_pdo.pos_ref = tx_pdo.vel_ref = tx_pdo.tor_ref = curr_ref=0.0;
    
    T::init();
    T::write_connect();
};

template < class T >
inline MotorPdo<T>::~MotorPdo()
{
   T::write_quit();
};


template class MotorPdo<EcPipePdo>;
template class MotorPdo<EcZmqPdo>;

#endif
