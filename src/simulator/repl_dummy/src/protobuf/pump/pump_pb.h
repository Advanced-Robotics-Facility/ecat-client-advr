#ifndef __PUMP_PB__
#define __PUMP_PB__


#include "../esc_pb.h"

class PumpPb : public EscPb{
    
private:
    std::default_random_engine gen;
    std::uniform_real_distribution<float> encoder_position{0.0,100.0};
    std::uniform_real_distribution<float> torque{0.0,50.0};
    std::uniform_real_distribution<float> temp{30.0,40.0};
    uint32_t press_demand;
    uint32_t    hpu_demand;
    uint32_t    vesc1mode;
    uint32_t    vesc2mode;
    uint32_t    fan1_speed;
    uint32_t    fan2_speed;
    uint16_t    dout;
public:
   
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb)
    {
        if ( pb.type() != iit::advr::Ec_slave_pdo::TX_HYQ_HPU ) {
            return;
        }

        press_demand=   pb.mutable_hyqhpu_tx_pdo()->demandpressure();
        hpu_demand=     pb.mutable_hyqhpu_tx_pdo()->hpudemandmode();
        vesc1mode=      pb.mutable_hyqhpu_tx_pdo()->vesc1mode();
        vesc2mode=      pb.mutable_hyqhpu_tx_pdo()->vesc2mode();
        fan1_speed=     pb.mutable_hyqhpu_tx_pdo()->fan1spd();
        fan2_speed=     pb.mutable_hyqhpu_tx_pdo()->fan2spd();
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
        // HPU_rx_pdo
        pb.mutable_hyqhpu_rx_pdo()->set_pressure(press_demand);
        //tempExpansionBoard miss
        pb.mutable_hyqhpu_rx_pdo()->set_statusword(0);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1boardtemp(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1mottemp(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2boardtemp(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2mottemp(temp(gen));
        
        //vesc1SwVer miss
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1actcur(0);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1actspd(fan1_speed);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1status(0);
        //vesc2SwVer miss
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2actcur(0);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2actspd(fan2_speed);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2status(0); 
        
        pb.mutable_hyqhpu_rx_pdo()->set_temp1(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_temp2(temp(gen));
        pb.mutable_hyqhpu_rx_pdo()->set_temp3(temp(gen));

        pb.mutable_hyqhpu_rx_pdo()->set_vesc1fbdutycycle(0);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2fbdutycycle(0);
        
        pb.mutable_hyqhpu_rx_pdo()->set_vesc1demand(vesc1mode);
        pb.mutable_hyqhpu_rx_pdo()->set_vesc2demand(vesc2mode);
        
        pb.mutable_hyqhpu_rx_pdo()->set_pwm1dutycycle(0);
        pb.mutable_hyqhpu_rx_pdo()->set_pwm2dutycycle(0);
    }
};

#endif
