#ifndef __NRT_ESC_ZMQ__
#define __NRT_ESC_ZMQ__

#include <utils.h>
#include "utils/ec_thread.h"

#include "zmq.hpp"
#include <zmq_addon.hpp>
#include <esc_info.h>

#include "protobuf/motor/motor_pb.h"
#include "protobuf/pow/pow_pb.h"
#include "protobuf/imu/imu_pb.h"
#include "protobuf/ft/ft_pb.h"
#include "protobuf/valve/valve_pb.h"
#include "protobuf/pump/pump_pb.h"

class EcPub{
private:
    std::shared_ptr<zmq::context_t> _context;
    std::shared_ptr<zmq::socket_t>  _publisher;
    std::string _zmq_uri;
    std::string _esc_name;

public:
    EcPub(const std::string zmq_uri,std::string esc_name):_zmq_uri(zmq_uri),_esc_name(esc_name){
        
        DPRINTF("PDO ZMQ URI: [%s]\n",_zmq_uri.c_str()); 

        int opt_linger = 1;
        #if ZMQ_VERSION_MAJOR == 2
            uint64_t opt_hwm = 1;
        #else
            int opt_hwm = 1;
        #endif

        _context = std::make_shared<zmq::context_t>(1);
        _publisher = std::make_shared<zmq::socket_t>(*_context, ZMQ_PUB);
        _publisher->setsockopt ( ZMQ_LINGER, &opt_linger, sizeof ( opt_linger ) );
        _publisher->setsockopt ( ZMQ_SNDHWM, &opt_hwm, sizeof ( opt_hwm ) );
        _publisher->bind(_zmq_uri);
    } 
    ~EcPub(){
        _publisher->disconnect(_zmq_uri);
    }

    std::string get_zmq_pub_uri()
    {
        return _zmq_uri;
    }

    int publish_msg(iit::advr::Ec_slave_pdo pdo) {
        std::string pb_msg_serialized;
        zmq::multipart_t multipart;
        try {
            pdo.SerializeToString(&pb_msg_serialized);
            multipart.push(zmq::message_t(pb_msg_serialized.c_str(), pb_msg_serialized.length()));
            multipart.push(zmq::message_t(_esc_name.c_str(), _esc_name.length()));
            multipart.send((*_publisher));
            printf("Esc_id [%s]send...{%ld}\n",_esc_name.c_str(),pb_msg_serialized.size());
        } catch ( zmq::error_t& e ) { 
            printf ( ">>> zsend ... catch %s\n", e.what() );
            return -1;
        }

        return 0;
    }   
};

class NrtEscZmq : public EcThread {

private:
    
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    std::map<int,std::shared_ptr<EscPb>> esc_pb;
    std::map<int,std::shared_ptr<EcPub>> esc_zmq_pub;
    std::map<int, iit::advr::Ec_slave_pdo>  pb_rx_pdos;
    
public:

    NrtEscZmq(std::map<int,int> esc_map,uint32_t th_period_us,
              std::string protocol,std::string host_address,uint32_t host_port){
        name="NrtEscZmq";
        //periodic
        struct timespec ts;
        iit::ecat::us2ts(&ts, th_period_us);
        //period.period is a timeval ... tv_usec 
        period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) / 2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

        if(host_address=="localhost"){
            host_address="127.0.0.1";
        }

        if(protocol=="zipc"){
            protocol="ipc";
        }
    
        for ( const auto& [id, esc_type] : esc_map ) {
            auto esc_name = iit::ecat::esc_type_map.at(esc_type);  

            std::string host_port_cmd = std::to_string(host_port+id);
            std::string zmq_uri = protocol+"://" + host_address + ":"+host_port_cmd;
            auto ec_pub= std::make_shared<EcPub>(zmq_uri,esc_name);
            esc_zmq_pub[id]=ec_pub;

            pb_rx_pdos[id] = iit::advr::Ec_slave_pdo();
            
            switch ( esc_type  )
            {
                case iit::ecat::CENT_AC :
                case iit::ecat::LO_PWR_DC_MC:{
                    auto hhcm_motor_pb= std::make_shared<HhcmMotor>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(hhcm_motor_pb);
                }break;
                case iit::ecat::SYNAPTICON_v5_0:
                case iit::ecat::SYNAPTICON_v5_1:{
                    auto circulo_motor_pb= std::make_shared<CirculoMotor>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(circulo_motor_pb);
                }break;
                case iit::ecat::AMC_FLEXPRO:{
                    auto flexpro_motor_pb= std::make_shared<FlexProMotor>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(flexpro_motor_pb);
                }break;
                case iit::ecat::FT6_MSP432:{
                    auto ft_pb= std::make_shared<FtPb>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(ft_pb);
                }break;   
                case iit::ecat::IMU_ANY :{
                    auto imu_pb= std::make_shared<ImuPb>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(imu_pb);
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                    auto pow_pb= std::make_shared<PowPb>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(pow_pb);
                }break;
                case iit::ecat::HYQ_KNEE:{
                    auto valve_pb= std::make_shared<ValvePb>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(valve_pb);
                }break;
                case iit::ecat::HYQ_HPU:{
                    auto pump_pb= std::make_shared<PumpPb>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(pump_pb);
                }break;
                
                default:
                    break;
            }           
        }
    }

    ~NrtEscZmq(){ 
        iit::ecat::print_stat ( s_loop );
    }

    virtual void th_init ( void * ){ 
        start_time = iit::ecat::get_time_ns();
        tNow = tPre = start_time;
        loop_cnt = 0;
    }
    
    virtual void th_loop ( void * ){

        for ( auto& [id, ec_pub] : esc_zmq_pub ) {
            if(esc_pb[id]!=nullptr){
                esc_pb[id]->pbSerialize(pb_rx_pdos[id]);  
                ec_pub->publish_msg(pb_rx_pdos[id]);
            }
        }
        
    }
    
};

#endif
