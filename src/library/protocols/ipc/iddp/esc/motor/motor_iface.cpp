#include "motor_iface.h"

motor_iface::motor_iface( const std::string robot_name, int32_t id, uint32_t type) :
    esc_pipe_iface(id, type, robot_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

motor_iface::motor_iface( const std::string robot_name, int32_t id) :
    esc_pipe_iface(id, "Motor", robot_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

motor_iface::motor_iface( int32_t id, uint32_t type, std::string rd_pp_name, std::string wr_pp_name) :
    esc_pipe_iface(id, type, rd_pp_name, wr_pp_name)
{
    pos_ref = vel_ref = tor_ref = 0.0;
}

void motor_iface::get_from_pb(void) 
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

void motor_iface::set_to_pb(void) 
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


