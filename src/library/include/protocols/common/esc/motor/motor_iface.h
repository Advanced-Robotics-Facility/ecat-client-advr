#ifndef __MOTOR_IFACE__
#define __MOTOR_IFACE__

#include<tuple>
#include <pb_utils.h>
#include "protocols/common/pipe/ec_pipe_pdo.h"
#include "protocols/common/zmq/ec_zmq_pdo.h"


typedef struct MOTOR_PDO_t{
    std::vector<std::string>ft_pb_name = {"force_x", "force_y", "force_z","torque_x", "torque_y", "torque_z"};
    
    std::tuple<float, float, float, float,   // pos_{link,motor}, vel_{link,motor}
               float,// torque
               float,float,                  // {motor,board}
               uint32_t, uint32_t,           // fault, rtt, op_idx_ack                  
               uint32_t, float, uint32_t> mt_t;  // aux // cmd_aux_sts
               
    
    // tx_pdo values
    float pos_ref, vel_ref, tor_ref, kp_ref, kd_ref;
    float tau_p_ref, tau_d_ref, tau_fc_ref;
    uint32_t aux_rd_idx_req, aux_wr_idx;
    float aux_wr;

    // rx_pdo values
    float   motor_pos, motor_vel, link_pos, link_vel, torque;
    float   motor_temperature, board_temperature;
    uint32_t aux_rd_idx_ack, fault,rtt,cmd_aux_sts;
    float aux_rd;
    float read_pos_ref, read_vel_ref, read_torque_ref;
    
}MOTOR_PDO;

template <class T>
class MotorPdo: public T{

public:

    MotorPdo(const std::string ,int32_t id);
    MotorPdo(const std::string ,int32_t id, uint32_t type);
    ~MotorPdo();

    void get_from_pb(void) override;

    void set_to_pb(void) override;
    
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


template < class T >
inline void MotorPdo<T>::get_from_pb() 
{
    _motor_pdo.link_pos            = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
    std::get<0>(_motor_pdo.mt_t)   = _motor_pdo.link_pos;
    _motor_pdo.motor_pos           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
    std::get<1>(_motor_pdo.mt_t)   = _motor_pdo.motor_pos;
    _motor_pdo.link_vel            = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
    std::get<2>(_motor_pdo.mt_t)   = _motor_pdo.link_vel;
    _motor_pdo.motor_vel           = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
    std::get<3>(_motor_pdo.mt_t)   = _motor_pdo.motor_vel;
    _motor_pdo.torque              = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
    std::get<4>(_motor_pdo.mt_t)   = _motor_pdo.torque;
    _motor_pdo.motor_temperature   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
    std::get<5>(_motor_pdo.mt_t)   = _motor_pdo.motor_temperature;
    _motor_pdo.board_temperature   = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();
    std::get<6>(_motor_pdo.mt_t)   = _motor_pdo.board_temperature;
    _motor_pdo.fault               = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
    std::get<7>(_motor_pdo.mt_t)   = _motor_pdo.fault;
    _motor_pdo.rtt                 = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt();
    std::get<8>(_motor_pdo.mt_t)   = _motor_pdo.rtt;
    _motor_pdo.aux_rd_idx_ack      = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack();
    std::get<9>(_motor_pdo.mt_t)   = _motor_pdo.aux_rd_idx_ack;
    _motor_pdo.aux_rd              = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux();
    std::get<10>(_motor_pdo.mt_t)  = _motor_pdo.aux_rd ;
    
    _motor_pdo.read_pos_ref        = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
    _motor_pdo.read_vel_ref        = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
    _motor_pdo.read_torque_ref     = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
    
    _motor_pdo.cmd_aux_sts         = T::pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts();
    std::get<11>(_motor_pdo.mt_t)  = _motor_pdo.cmd_aux_sts ;
}

template < class T >
inline void MotorPdo<T>::set_to_pb() 
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);
    // Type
    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    //
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref(_motor_pdo.pos_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_vel_ref(_motor_pdo.vel_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_tor_ref(_motor_pdo.tor_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0 (_motor_pdo.kp_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1 (_motor_pdo.kd_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2 (_motor_pdo.tau_p_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3 (_motor_pdo.tau_d_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4 (_motor_pdo.tau_fc_ref);
    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);

    T::pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(_motor_pdo.aux_rd_idx_req);
}

template class MotorPdo<EcPipePdo>;
template class MotorPdo<EcZmqPdo>;

#endif
