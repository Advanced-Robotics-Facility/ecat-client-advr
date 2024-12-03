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
extern zmq::context_t zmq_ctx;

class EcPub{
private:
    std::shared_ptr<zmq::socket_t>  _publisher;
    std::string _zmq_uri;
    std::string _esc_name;
    zmq::message_t  _msg_id;
    zmq::message_t  _msg;
    std::shared_ptr<zmq::context_t>  _pub_context;

public:
    EcPub(const std::string zmq_uri,std::string esc_name,std::shared_ptr<zmq::context_t> pub_context):
        _zmq_uri(zmq_uri),_esc_name(esc_name),_pub_context(pub_context){
        
        DPRINTF("PDO ZMQ URI: [%s]\n",_zmq_uri.c_str()); 

        int opt_linger = 1;
        #if ZMQ_VERSION_MAJOR == 2
            uint64_t opt_hwm = 1;
        #else
            int opt_hwm = 1;
        #endif
        try{
            // prepare _msg_id just once
            _msg_id.rebuild ( _esc_name.length() );
            memcpy ((void*)_msg_id.data(), _esc_name.data(), _esc_name.length());

            _publisher = std::make_shared<zmq::socket_t>(zmq_ctx,ZMQ_PUB);
            _publisher->setsockopt ( ZMQ_LINGER, &opt_linger, sizeof ( opt_linger ) );
            _publisher->setsockopt ( ZMQ_SNDHWM, &opt_hwm, sizeof ( opt_hwm ) );
            _publisher->bind(_zmq_uri);
        }catch ( zmq::error_t& e ) { 
            std::string zmq_error(e.what());
            throw std::runtime_error("error on publisher socket initialization: "+zmq_error);
        }
    } 
    ~EcPub(){
        try{
            _publisher->unbind(_zmq_uri);
            _publisher->close();
        }catch ( zmq::error_t& e ) { 
            std::cout << "error on closing phase: " << e.what() << std::endl;
        }
    }

    std::string get_zmq_pub_uri()
    {
        return _zmq_uri;
    }

    int publish_msg(iit::advr::Ec_slave_pdo &pdo_msg) {
        try {
            if ( ! pdo_msg.IsInitialized() ) {
                DPRINTF("msg is NOT initialized\n");
                return -EINVAL;
            }

            size_t msg_size = pdo_msg.ByteSize();
            if ( msg_size+sizeof(msg_size) > MAX_PB_SIZE ) {
                DPRINTF("msg_size TOO big %ld > %d \n", msg_size, MAX_PB_SIZE);
                return -EOVERFLOW;
            }
            _msg.rebuild ( msg_size );
            pdo_msg.SerializeToArray(_msg.data(), msg_size);

            _publisher->send ( _msg_id, ZMQ_SNDMORE );
            _publisher->send ( _msg, 0);

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
    std::shared_ptr<zmq::context_t>  _pub_context;
    std::map<int,int> _esc_map;
    std::string _protocol,_host_address;
    uint32_t _host_port;
public:

    NrtEscZmq(std::map<int,int> esc_map,uint32_t th_period_us,
              std::string protocol,std::string host_address,uint32_t host_port):
              _esc_map(esc_map),_protocol(protocol),_host_address(host_address),_host_port(host_port){
        name="NrtEscZmq";
        //periodic
        struct timespec ts;
        iit::ecat::us2ts(&ts, th_period_us);
        //period.period is a timeval ... tv_usec 
        period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) / 2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

        if(_host_address=="localhost"){
            _host_address="127.0.0.1";
        }

        if(_protocol=="zipc"){
            _protocol="ipc";
        }
    }

    ~NrtEscZmq(){ 
        iit::ecat::print_stat ( s_loop );
        for ( auto& [id, ec_pub] : esc_zmq_pub ) {
            ec_pub.reset();
        }
        _pub_context->close();
    }

    virtual void th_init ( void * ){ 

        _pub_context=std::make_shared<zmq::context_t>(1);

        for ( const auto& [id, esc_type] : _esc_map ) {
            auto esc_name = iit::ecat::esc_type_map.at(esc_type);  

            std::string host_port_cmd = std::to_string(_host_port+id);
            std::string zmq_uri = _protocol+"://" + _host_address + ":"+host_port_cmd;
            esc_zmq_pub[id]=std::make_shared<EcPub>(zmq_uri,esc_name,_pub_context);;

            pb_rx_pdos[id] = iit::advr::Ec_slave_pdo();
            
            switch ( esc_type  )
            {
                case iit::ecat::CENT_AC :
                case iit::ecat::LO_PWR_DC_MC:{
                    auto advrf_motor_pb= std::make_shared<AdvrfMotor>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(advrf_motor_pb);
                }break;
                case iit::ecat::SYNAPTICON_v5_0:
                case iit::ecat::SYNAPTICON_v5_1:{
                    auto synapticon_motor_pb= std::make_shared<SynapticonMotor>();
                    esc_pb[id]=std::static_pointer_cast<EscPb>(synapticon_motor_pb);
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
