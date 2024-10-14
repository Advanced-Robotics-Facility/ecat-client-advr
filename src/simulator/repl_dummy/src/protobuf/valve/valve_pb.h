#ifndef __VALVE_PB__
#define __VALVE_PB__


#include "../esc_pb.h"

class ValvePb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> encoder_position{0.0,100.0};
    std::uniform_real_distribution<float> force{0.0,50.0};
    std::uniform_real_distribution<float> temp{30.0,40.0};
    float curr_ref,position_ref,force_ref;

public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_HYQ_KNEE ) {
            return;
        }
        curr_ref=           pb.mutable_hyqknee_tx_pdo()->current_ref();
        position_ref=       pb.mutable_hyqknee_tx_pdo()->position_ref();
        force_ref=          pb.mutable_hyqknee_tx_pdo()->force_ref();
    }
    
    virtual void  pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb.set_type ( iit::advr::Ec_slave_pdo::RX_HYQ_KNEE );
        // HyqKnee_rx_pdo
        pb.mutable_hyqknee_rx_pdo()->set_encoder_position(encoder_position(gen));
        pb.mutable_hyqknee_rx_pdo()->set_force (force(gen));
        pb.mutable_hyqknee_rx_pdo()->set_pressure_1(100*curr_ref);
        pb.mutable_hyqknee_rx_pdo()->set_pressure_2(100*curr_ref);
        pb.mutable_hyqknee_rx_pdo()->set_current(curr_ref);
        pb.mutable_hyqknee_rx_pdo()->set_temperature(temp(gen));
        pb.mutable_hyqknee_rx_pdo()->set_rtt(0.0);
        pb.mutable_hyqknee_rx_pdo()->set_aux(0.0);
        pb.mutable_hyqknee_rx_pdo()->set_current_ref_fb(curr_ref);
        pb.mutable_hyqknee_rx_pdo()->set_position_ref_fb(position_ref);
        pb.mutable_hyqknee_rx_pdo()->set_force_ref_fb(force_ref);
    }
};

#endif
