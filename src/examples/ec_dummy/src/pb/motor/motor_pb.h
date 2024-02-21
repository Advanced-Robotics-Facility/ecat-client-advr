#ifndef __MOTOR_PB__
#define __MOTOR_PB__


#include "../esc_pb.h"

class MotorPb : public EscPb{

private:
    float pos_ref,vel_ref,tor_ref;
    
public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        pos_ref=pb.mutable_motor_xt_tx_pdo()->pos_ref();
        vel_ref=pb.mutable_motor_xt_tx_pdo()->vel_ref();
        tor_ref=pb.mutable_motor_xt_tx_pdo()->tor_ref();
    }
    
    virtual void  pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        
        clock_gettime(CLOCK_MONOTONIC, &ts);

        // Header
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb.set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
        // Motor_xt_tx_pdo
        pb.mutable_motor_xt_rx_pdo()->set_link_pos(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_motor_pos(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_link_vel(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_motor_vel(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_torque(tor_ref);
        pb.mutable_motor_xt_rx_pdo()->set_temperature(0.0);
        pb.mutable_motor_xt_rx_pdo()->set_fault(0);
        pb.mutable_motor_xt_rx_pdo()->set_rtt(0);
        // optional
        pb.mutable_motor_xt_rx_pdo()->set_op_idx_ack(0);
        pb.mutable_motor_xt_rx_pdo()->set_aux(0);
        pb.mutable_motor_xt_rx_pdo()->set_motor_temp(0.0);
        pb.mutable_motor_xt_rx_pdo()->set_board_temp(0.0);
        //
        pb.mutable_motor_xt_rx_pdo()->set_cmd_aux_sts(0.0);
        
        pb.mutable_motor_xt_rx_pdo()->set_pos_ref(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_vel_ref(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_tor_ref(tor_ref);
    } 
};

#endif