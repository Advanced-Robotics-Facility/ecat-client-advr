#ifndef __GRIPPER_PB__
#define __GRIPPER_PB__

#include "../esc_pb.h"

class GripperPb : public EscPb {
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> position{0.0,8.3};

    float target_pos, target_vel, target_torque;
    float options[5]={0,0,0,0,0};

public:
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_GRIPPER_PDO ) {
            return;
        }

        auto* gripper_tx_pdo = pb.mutable_gripper_tx_pdo();
        target_pos = gripper_tx_pdo->target_pos();
        target_vel = gripper_tx_pdo->target_vel();
        target_torque = gripper_tx_pdo->target_torque();
        options[0] = gripper_tx_pdo->gain_0();
        options[1] = gripper_tx_pdo->gain_1();
        options[2] = gripper_tx_pdo->gain_2();
        options[3] = gripper_tx_pdo->gain_3();
        options[4] = gripper_tx_pdo->gain_4();

        return;
    }
    
    virtual void pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        pb.set_type(iit::advr::Ec_slave_pdo::RX_GRIPPER_PDO);

        auto* gripper_rx_pdo = pb.mutable_gripper_rx_pdo();
        gripper_rx_pdo->set_motor_pos(position(gen));
        gripper_rx_pdo->set_link_pos(position(gen));
        gripper_rx_pdo->set_demanded_pos(0);
        gripper_rx_pdo->set_demanded_vel(0);
        gripper_rx_pdo->set_demanded_torque(0);
        gripper_rx_pdo->set_error_code(0);

        return;
    }
};

#endif