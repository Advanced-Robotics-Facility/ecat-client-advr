#ifndef __RT_ESC_PIPE__
#define __RT_ESC_PIPE__

#include <utils.h>
#include <ipc_pipe.h>
#include "utils/ec_thread.h"
#include <pb_utils.h>
#include <esc_info.h>

#include "protobuf/motor/motor_pb.h"
#include "protobuf/pow/pow_pb.h"
#include "protobuf/imu/imu_pb.h"
#include "protobuf/ft/ft_pb.h"
#include "protobuf/valve/valve_pb.h"
#include "protobuf/pump/pump_pb.h"

#define POOL_SIZE   4096
#define MAX_WRK     64

class RtEscPipe : public EcThread {

private:
    
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;

    std::map<int,std::string> rd_esc_pdo;
    std::map<int,std::string> wr_esc_pdo;
    std::map<int,std::shared_ptr<EscPb>> esc_pb;

    std::map<int, IDDP_pipe>    rd_iddp, wr_iddp;
    std::map<int,bool> wr_iddp_connected;
    
    std::map<int, iit::advr::Ec_slave_pdo>  pb_rx_pdos;
    std::map<int, iit::advr::Ec_slave_pdo>  pb_tx_pdos;
    uint8_t                                 pb_buf[MAX_PB_SIZE];  
    
public:

    RtEscPipe(std::string robot_name,std::map<int,int> esc_map,uint32_t th_period_us ){
        name="RtEscPipe";
        //periodic
        struct timespec ts;
        iit::ecat::us2ts(&ts, th_period_us);
        //period.period is a timeval ... tv_usec 
        period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
#ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_OTHER;
#endif
        priority = sched_get_priority_max ( schedpolicy ) / 2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
        for ( const auto& [id, esc_type] : esc_map ) {
            auto esc_name = iit::ecat::esc_type_map.at(esc_type);   
            std::string rd_pp_name = make_pipe_name(robot_name,esc_name,id,"rx");
            std::string wr_pp_name = make_pipe_name(robot_name,esc_name,id,"tx");
            rd_esc_pdo[id]=rd_pp_name;
            wr_esc_pdo[id]=wr_pp_name;
            
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
        
        for ( const auto& [id, rd_name] : rd_esc_pdo ) {
            DPRINTF("%d %s %s\n", id, rd_name.c_str(), wr_esc_pdo[id].c_str());
        }
    }

    ~RtEscPipe(){ 
        iit::ecat::print_stat ( s_loop );
    }

    virtual void th_init ( void * ){
        start_time = iit::ecat::get_time_ns();
        tNow = tPre = start_time;
        loop_cnt = 0;
        
        // bind rd iddp
        for ( const auto& [id, pipe_name] : rd_esc_pdo ) {
            rd_iddp[id] = IDDP_pipe();
            rd_iddp[id].bind(pipe_name, POOL_SIZE);
        }
        
        // 
        for ( const auto& [id, pipe_name] : rd_esc_pdo ) {
            pb_rx_pdos[id] = iit::advr::Ec_slave_pdo();
            pb_tx_pdos[id] = iit::advr::Ec_slave_pdo();
        }

        // connect wr iddp
        for ( const auto& [id, not_used] : wr_esc_pdo ) {
            wr_iddp[id] = IDDP_pipe();
            wr_iddp_connected[id]=false;
        }

        // sanity check
        assert( (rd_iddp.size()==wr_iddp.size()) && (pb_tx_pdos.size() == pb_rx_pdos.size()) );     
    }
    
    virtual void th_loop ( void * ){

        // etherCAT rx
        for ( auto& [id, pipe] : wr_iddp ) {
            if (wr_iddp_connected[id] ){
                if(esc_pb[id]!=nullptr){
                    esc_pb[id]->pbSerialize(pb_rx_pdos[id]);    
                    write_pb_to(pipe, pb_buf, sizeof(pb_buf), &pb_rx_pdos[id], name); // write pipe
                }
            }
        }

        for ( auto& [id, pipe] : rd_iddp ) {
            int32_t nbytes = 0;
            do {
                nbytes = read_pb_from(pipe, pb_buf, sizeof(pb_buf), &pb_tx_pdos[id], name);
                if ( nbytes  > 0 ) {
                    if ( pb_tx_pdos[id].type() == iit::advr::Ec_slave_pdo::CLIENT_PIPE ) {
                        auto pb_client_pdo = pb_tx_pdos[id].mutable_client_pdo();
                        if ( pb_client_pdo->type() == iit::advr::Client_pipe::CONNECT ) {
                            if ( ! wr_iddp_connected[id] ) { 
                                wr_iddp[id].connect( wr_esc_pdo[id]);
                                wr_iddp_connected[id] = true;
                                DPRINTF ("connect to %s_tx_pdo port\n", wr_esc_pdo[id].c_str() );
                            } 
                            else{
                                //DPRINTF (" Already connected to %s_tx_pdo\n", wr_esc_pdo[id].c_str() );                                
                            }
                        } 
                        else if ( pb_client_pdo->type() == iit::advr::Client_pipe::QUIT ) {
                            if ( wr_iddp_connected[id] ) {
                                wr_iddp[id].close_pipe();
                                wr_iddp_connected[id] = false;
                                DPRINTF ("quit %s_tx_pdo\n", wr_esc_pdo[id].c_str() );
                            } 
                            else {
                                //DPRINTF (" Already disconnected to %s_tx_pdo\n", wr_esc_pdo[id].c_str() );
                            }               
                        }
                    } 
                    else {
                        if(esc_pb[id]!=nullptr){
                            esc_pb[id]->pbDeserialize(pb_tx_pdos[id]);  
                        }
                    }
                } else {                
                }
            } while (nbytes > 0);
            // read pipe
        }      
        //ethercat tx       
    }
    
};

#endif
