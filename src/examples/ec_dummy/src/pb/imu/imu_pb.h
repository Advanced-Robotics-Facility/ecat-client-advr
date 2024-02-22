#ifndef __IMU_PB__
#define __IMU_PB__


#include "../esc_pb.h"

class ImuPb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> rate{0,3};
    std::uniform_real_distribution<float> acc{0,2};
    std::uniform_real_distribution<float> quat{0,1};
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
        pb.set_type(iit::advr::Ec_slave_pdo::RX_IMU_VN);
        // ImuVN_rx_pdo
        pb.mutable_imuvn_rx_pdo()->set_x_rate(rate(gen));
        pb.mutable_imuvn_rx_pdo()->set_y_rate(rate(gen));
        pb.mutable_imuvn_rx_pdo()->set_z_rate(rate(gen));
        pb.mutable_imuvn_rx_pdo()->set_x_acc(acc(gen));
        pb.mutable_imuvn_rx_pdo()->set_y_acc(acc(gen));
        pb.mutable_imuvn_rx_pdo()->set_z_acc(acc(gen));
        pb.mutable_imuvn_rx_pdo()->set_x_quat(quat(gen));
        pb.mutable_imuvn_rx_pdo()->set_y_quat(quat(gen));
        pb.mutable_imuvn_rx_pdo()->set_z_quat(quat(gen));
        pb.mutable_imuvn_rx_pdo()->set_w_quat(quat(gen));
        pb.mutable_imuvn_rx_pdo()->set_imu_ts(0);
        pb.mutable_imuvn_rx_pdo()->set_temperature(temp(gen));
        pb.mutable_imuvn_rx_pdo()->set_digital_in(0);
        pb.mutable_imuvn_rx_pdo()->set_fault(0);
        pb.mutable_imuvn_rx_pdo()->set_rtt(0);
    }
};


#endif
