#ifndef __FT_PB__
#define __FT_PB__


#include "../esc_pb.h"

class FtPb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> force{0,100};
    std::uniform_real_distribution<float> torque{0,10};
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
        pb.set_type(iit::advr::Ec_slave_pdo::RX_FT6);
        // FT6_rx_pdo
        pb.mutable_ft6_rx_pdo()->set_force_x(force(gen));
        pb.mutable_ft6_rx_pdo()->set_force_y(force(gen));
        pb.mutable_ft6_rx_pdo()->set_force_z(force(gen));
        pb.mutable_ft6_rx_pdo()->set_torque_x(torque(gen));
        pb.mutable_ft6_rx_pdo()->set_torque_y(torque(gen));
        pb.mutable_ft6_rx_pdo()->set_torque_z(torque(gen));
        pb.mutable_ft6_rx_pdo()->set_fault(0);
        pb.mutable_ft6_rx_pdo()->set_rtt(0);
    }
};

#endif
