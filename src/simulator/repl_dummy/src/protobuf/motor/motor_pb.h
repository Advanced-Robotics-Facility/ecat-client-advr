#ifndef __MOTOR_PB__
#define __MOTOR_PB__


#include "../esc_pb.h"

class AdvrfMotor : public EscPb{

private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> temp{30.0,40.0};
    float pos_ref,vel_ref,tor_ref,curr_ref;
public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_XT_MOTOR ) {
            return;
        }
        pos_ref=pb.mutable_motor_xt_tx_pdo()->pos_ref();
        vel_ref=pb.mutable_motor_xt_tx_pdo()->vel_ref();
        tor_ref=pb.mutable_motor_xt_tx_pdo()->tor_ref();
        curr_ref=pb.mutable_motor_xt_tx_pdo()->tor_ref();
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
        // no status word
        pb.mutable_motor_xt_rx_pdo()->set_link_pos(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_motor_pos(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_link_vel(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_motor_vel(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_torque(tor_ref);
        pb.mutable_motor_xt_rx_pdo()->set_aux(curr_ref);
        pb.mutable_motor_xt_rx_pdo()->set_motor_temp(temp(gen));
        pb.mutable_motor_xt_rx_pdo()->set_board_temp(temp(gen));
        pb.mutable_motor_xt_rx_pdo()->set_fault(0);
        pb.mutable_motor_xt_rx_pdo()->set_rtt(0);
        pb.mutable_motor_xt_rx_pdo()->set_pos_ref(pos_ref);
        pb.mutable_motor_xt_rx_pdo()->set_vel_ref(vel_ref);
        pb.mutable_motor_xt_rx_pdo()->set_tor_ref(tor_ref);
        // no current feedback
        pb.mutable_motor_xt_rx_pdo()->set_temperature(0.0); //added here!!! it's not optional
    } 
};


class SynapticonMotor : public EscPb{

private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> temp{30.0,40.0};
    float pos_ref,vel_ref,tor_ref,curr_ref;
    
public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_CIA402 ) {
            return;
        }
        pos_ref=pb.mutable_cia402_tx_pdo()->target_pos();
        vel_ref=pb.mutable_cia402_tx_pdo()->target_vel();
        tor_ref=pb.mutable_cia402_tx_pdo()->target_torque();
        curr_ref=pb.mutable_cia402_tx_pdo()->target_current();
    }
    
    virtual void  pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb.set_type(iit::advr::Ec_slave_pdo::RX_CIA402);
        
        pb.mutable_cia402_rx_pdo()->set_statusword(0);
        pb.mutable_cia402_rx_pdo()->set_link_pos(pos_ref);
        pb.mutable_cia402_rx_pdo()->set_motor_pos(pos_ref);
        pb.mutable_cia402_rx_pdo()->set_link_vel(vel_ref);
        pb.mutable_cia402_rx_pdo()->set_motor_vel(vel_ref);
        pb.mutable_cia402_rx_pdo()->set_torque(tor_ref);
        pb.mutable_cia402_rx_pdo()->set_current(curr_ref);
        pb.mutable_cia402_rx_pdo()->set_motor_temp(temp(gen));
        pb.mutable_cia402_rx_pdo()->set_drive_temp(temp(gen));
        pb.mutable_cia402_rx_pdo()->set_error_code(0);
        pb.mutable_cia402_rx_pdo()->set_demanded_pos(pos_ref);
        pb.mutable_cia402_rx_pdo()->set_demanded_vel(vel_ref);
        pb.mutable_cia402_rx_pdo()->set_demanded_torque(tor_ref);
        pb.mutable_cia402_rx_pdo()->set_demanded_current(curr_ref);
    } 
};

#endif
