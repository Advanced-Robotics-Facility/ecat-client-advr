#ifndef __POW_PB__
#define __POW_PB__


#include "../esc_pb.h"

class PowPb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> volt{48.0,50.0};
    std::uniform_real_distribution<float> curr{5.0,10.0};
    std::uniform_real_distribution<float> temp{30.0,40.0};
public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {

    }
    
    virtual void  pbSerialize(iit::advr::Ec_slave_pdo& pb)
    {
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb.set_type(iit::advr::Ec_slave_pdo::RX_POW_F28M36);
        // PowF28M36_rx_pdo
        pb.mutable_powf28m36_rx_pdo()->set_v_batt(volt(gen));
        pb.mutable_powf28m36_rx_pdo()->set_v_load(volt(gen));
        pb.mutable_powf28m36_rx_pdo()->set_i_load(curr(gen));
        pb.mutable_powf28m36_rx_pdo()->set_temp_batt(temp(gen));
        pb.mutable_powf28m36_rx_pdo()->set_temp_heatsink(temp(gen));
        pb.mutable_powf28m36_rx_pdo()->set_temp_pcb(temp(gen));
        pb.mutable_powf28m36_rx_pdo()->set_status(0);
        pb.mutable_powf28m36_rx_pdo()->set_fault(0);
        pb.mutable_powf28m36_rx_pdo()->set_rtt(0);
        //
        pb.mutable_powf28m36_rx_pdo()->set_op_idx_ack(0);
        pb.mutable_powf28m36_rx_pdo()->set_aux(0);
    }
};

#endif
