#ifndef __MOTOR_PDO__
#define __MOTOR_PDO__

#include<tuple>
#include <pb_utils.h>
#include "common/pipe/ec_pipe_pdo.h"
#include "common/zmq/ec_zmq_pdo.h"


typedef struct MOTOR_PDO_t{
    std::vector<std::string>motor_pb_name = {"link_pos", "motor_pos", "link_vel",
                                             "motor_vel", "torque", "motor_temp",
                                             "board_temp","fault","rtt","op_idx_ack","aux","cmd_aux_sts"};
    
    std::tuple<float, float, float, float,   // pos_{link,motor}, vel_{link,motor}
               float,// torque
               float,float,                  // {motor,board}
               uint32_t, uint32_t,           // fault, rtt, op_idx_ack                  
               uint32_t, float, uint32_t> mt_t;  // aux // cmd_aux_sts
               
    
    // tx_pdo values
    float pos_ref, vel_ref, tor_ref,curr_ref, kp_ref, kd_ref;
    float tau_p_ref, tau_d_ref, tau_fc_ref;
    uint32_t aux_rd_idx_req, aux_wr_idx;
    float aux_wr;

    // rx_pdo values
    float   motor_pos, motor_vel, link_pos, link_vel, torque;
    float   motor_temperature, board_temperature;
    uint32_t aux_rd_idx_ack, fault,rtt,cmd_aux_sts;
    float aux_rd;
    float read_pos_ref, read_vel_ref, read_torque_ref,read_curr_ref;
    
}MOTOR_PDO;

template <class T>
class MotorPdo: public T{

public:

    MotorPdo(const std::string ,int32_t id);
    MotorPdo(const std::string ,int32_t id, uint32_t type);
    ~MotorPdo();

    virtual void get_from_pb(void)=0;

    virtual void set_to_pb(void)=0;
    
    MOTOR_PDO _motor_pdo;

};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int id):
                           T(id,"Motor",value)
{
    _motor_pdo.pos_ref = _motor_pdo.vel_ref = _motor_pdo.tor_ref = 0.0;
    
    T::init();
    T::write_connect();
};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int32_t id, uint32_t type):
                           T(id, type, value)
{
    _motor_pdo.pos_ref = _motor_pdo.vel_ref = _motor_pdo.tor_ref = 0.0;
    
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
