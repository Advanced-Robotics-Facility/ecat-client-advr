#ifndef __MOTOR_IFACE__
#define __MOTOR_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"


/**
 * @brief The motor_iface class implements the generic
 * esc_iface to handle an esc of type motor
 */
class motor_iface: public esc_pipe_iface {

public:

    /**
     * @brief motor_iface constructor
     * @param robot_name according to ecat master convention
     * @param id is the slave id
     */
    motor_iface( int32_t id, uint32_t type,
                 std::string rd_pp_name,
                 std::string wr_pp_name );

    motor_iface( const std::string ,int32_t id, uint32_t type);
    
    motor_iface( const std::string ,int32_t id);

    void get_from_pb(void) override;

    void set_to_pb(void) override;

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
    float read_pos_ref, read_vel_ref, read_torque_ref;  // ack from master    
};


inline motor_iface::motor_iface( const std::string robot_name, int32_t id, uint32_t type) :
    esc_pipe_iface(id, type, robot_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

inline motor_iface::motor_iface( const std::string robot_name, int32_t id) :
    esc_pipe_iface(id, "Motor", robot_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

inline motor_iface::motor_iface( int32_t id, uint32_t type, std::string rd_pp_name, std::string wr_pp_name) :
    esc_pipe_iface(id, type, rd_pp_name, wr_pp_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

inline void motor_iface::get_from_pb(void) 
{
        
    motor_pos           = pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos();
    motor_vel           = pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel();
    link_pos            = pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos();
    link_vel            = pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel();
    torque              = pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque();
    motor_temperature   = pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp();
    board_temperature   = pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp();

    aux_rd_idx_ack      = pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack();
    aux_rd              = pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux();

    fault               = pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault();
    rtt                 = pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt();

    read_pos_ref        = pb_rx_pdos.mutable_motor_xt_rx_pdo()->pos_ref();
    read_vel_ref        = pb_rx_pdos.mutable_motor_xt_rx_pdo()->vel_ref();
    read_torque_ref     = pb_rx_pdos.mutable_motor_xt_rx_pdo()->tor_ref();
    cmd_aux_sts         = pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts();
        
}

inline void motor_iface::set_to_pb(void) 
{

    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    // Type
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_XT_MOTOR);
    //
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_pos_ref(pos_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_vel_ref(vel_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_tor_ref(tor_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_ts(uint32_t(iit::ecat::get_time_ns()/1000));

    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_0 (kp_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_1 (kd_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_2 (tau_p_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_3 (tau_d_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_gain_4 (tau_fc_ref);
    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_fault_ack(0);

    pb_tx_pdos.mutable_motor_xt_tx_pdo()->set_op_idx_aux(aux_rd_idx_req);
             
}


#endif
