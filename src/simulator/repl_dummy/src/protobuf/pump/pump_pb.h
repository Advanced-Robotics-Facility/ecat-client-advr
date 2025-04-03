#ifndef __PUMP_PB__
#define __PUMP_PB__


#include "../esc_pb.h"

class PumpPb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> current{0.0,10.0};
    std::uniform_real_distribution<float> velocity{0.0,50.0};
    std::uniform_real_distribution<float> temp{30.0,40.0};
    float   pump_target;
    float   options[4]={0,0,0,0};
    float   aux=0;

public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_HYQ_HPU ) {
            return;
        }

        pump_target=    pb.mutable_hyqhpu_tx_pdo()->pump_target();
        options[0]=     pb.mutable_hyqhpu_tx_pdo()->pressure_p_gain();
        options[1]=     pb.mutable_hyqhpu_tx_pdo()->pressure_i_gain();
        options[2]=     pb.mutable_hyqhpu_tx_pdo()->pressure_d_gain();
        options[3]=     pb.mutable_hyqhpu_tx_pdo()->pressure_i_gain();
        aux=            pb.mutable_hyqhpu_tx_pdo()->aux();    
    }
    
    virtual void  pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb.set_type ( iit::advr::Ec_slave_pdo::RX_HYQ_HPU );

        pb.mutable_hyqhpu_rx_pdo()->set_motor_current(current(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_motor_speed(velocity(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_pressure1(pump_target);
        pb.mutable_hyqhpu_rx_pdo()->set_pressure2(pump_target);
        pb.mutable_hyqhpu_rx_pdo()->set_temperature(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_mosfet_temperature(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_motor_temperature(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_fault(0);
        pb.mutable_hyqhpu_rx_pdo()->set_rtt(0);
        pb.mutable_hyqhpu_rx_pdo()->set_op_idx_ack(0);
        pb.mutable_hyqhpu_rx_pdo()->set_aux(0); 
    }
};

#endif
