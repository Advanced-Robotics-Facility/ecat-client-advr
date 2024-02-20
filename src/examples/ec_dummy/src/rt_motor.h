#ifndef __RT_MOTOR__
#define __RT_MOTOR__

#include <utils.h>
#include <ipc_pipe.h>
#include <thread_util.h>
#include <pb_utils.h>

#include <protobuf/repl_cmd.pb.h>
#include <protobuf/ecat_pdo.pb.h>
#include <esc_info.h>

#define POOL_SIZE   4096
#define MAX_WRK     64

static float pos_ref,vel_ref,tor_ref;

class RtMotor : public Thread_hook {

private:
    
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;

    std::map<int,std::string> rd_motor_pdo;
    std::map<int,std::string> wr_motor_pdo;

    std::map<int, IDDP_pipe>    rd_iddp, wr_iddp;
    std::map<int,bool> wr_iddp_connected;
    
    std::map<int, iit::advr::Ec_slave_pdo>  pb_rx_pdos;
    std::map<int, iit::advr::Ec_slave_pdo>  pb_tx_pdos;
    uint8_t                                 pb_buf[MAX_PB_SIZE];  
    
    void pbDeserialize(int id)
    {
        auto pb = &pb_tx_pdos[id];
        pos_ref=pb->mutable_motor_xt_tx_pdo()->pos_ref();
        vel_ref=pb->mutable_motor_xt_tx_pdo()->vel_ref();
        tor_ref=pb->mutable_motor_xt_tx_pdo()->tor_ref();
    }
    
    void  pbSerialize(int id)
    {
        static struct timespec ts;
        auto pb = &pb_rx_pdos[id];
        
        auto pb_tx = &pb_tx_pdos[id];
        
        pos_ref=pb_tx->mutable_motor_xt_tx_pdo()->pos_ref();
        vel_ref=pb_tx->mutable_motor_xt_tx_pdo()->vel_ref();
        tor_ref=pb_tx->mutable_motor_xt_tx_pdo()->tor_ref();
        
        clock_gettime(CLOCK_MONOTONIC, &ts);

        // Header
        pb->mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb->mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb->set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
        // Motor_xt_tx_pdo
        pb->mutable_motor_xt_rx_pdo()->set_link_pos(pos_ref);
        pb->mutable_motor_xt_rx_pdo()->set_motor_pos(pos_ref);
        pb->mutable_motor_xt_rx_pdo()->set_link_vel(vel_ref);
        pb->mutable_motor_xt_rx_pdo()->set_motor_vel(vel_ref);
        pb->mutable_motor_xt_rx_pdo()->set_torque(tor_ref);
        pb->mutable_motor_xt_rx_pdo()->set_temperature(0.0);
        pb->mutable_motor_xt_rx_pdo()->set_fault(0);
        pb->mutable_motor_xt_rx_pdo()->set_rtt(0);
        // optional
        pb->mutable_motor_xt_rx_pdo()->set_op_idx_ack(0);
        pb->mutable_motor_xt_rx_pdo()->set_aux(0);
        pb->mutable_motor_xt_rx_pdo()->set_motor_temp(0.0);
        pb->mutable_motor_xt_rx_pdo()->set_board_temp(0.0);
        //
        pb->mutable_motor_xt_rx_pdo()->set_cmd_aux_sts(0.0);
        
        pb->mutable_motor_xt_rx_pdo()->set_pos_ref(pos_ref);
        pb->mutable_motor_xt_rx_pdo()->set_vel_ref(vel_ref);
        pb->mutable_motor_xt_rx_pdo()->set_tor_ref(tor_ref);
    }
    
    
public:

    RtMotor(std::string robot_name,std::map<int,int> motor_map,uint32_t th_period_us ){
        name="RtMotor";
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
    
        for ( const auto& [id, type] : motor_map ) {
            auto esc_name = iit::ecat::esc_type_map.at(type);   
            std::string rd_pp_name = make_pipe_name(robot_name,esc_name,id,"rx");
            std::string wr_pp_name = make_pipe_name(robot_name,esc_name,id,"tx");
            rd_motor_pdo[id]=rd_pp_name;
            wr_motor_pdo[id]=wr_pp_name;
        }
        
        for ( const auto& [id, rd_name] : rd_motor_pdo ) {
            DPRINTF("%d %s %s\n", id, rd_name.c_str(), wr_motor_pdo[id].c_str());
        }
        
        pos_ref=vel_ref=tor_ref=0.0;
    }

    ~RtMotor(){ 
        iit::ecat::print_stat ( s_loop );
    }

    virtual void th_init ( void * ){
        start_time = iit::ecat::get_time_ns();
        tNow = tPre = start_time;
        loop_cnt = 0;
        
        // bind rd iddp
        for ( const auto& [id, pipe_name] : rd_motor_pdo ) {
            rd_iddp[id] = IDDP_pipe();
            rd_iddp[id].bind(pipe_name, POOL_SIZE);
        }
        
        // 
        for ( const auto& [id, pipe_name] : rd_motor_pdo ) {
            pb_rx_pdos[id] = iit::advr::Ec_slave_pdo();
            pb_tx_pdos[id] = iit::advr::Ec_slave_pdo();
        }

        // connect wr iddp
        for ( const auto& [id, not_used] : wr_motor_pdo ) {
            wr_iddp[id] = IDDP_pipe();
            wr_iddp_connected[id]=false;
        }

        // sanity check
        assert( (rd_iddp.size()==wr_iddp.size()) && (pb_tx_pdos.size() == pb_rx_pdos.size()) );     
    }
    
    virtual void th_loop ( void * ){
        for ( auto& [id, pipe] : rd_iddp ) {
            int32_t nbytes = 0;
            do {
                nbytes = read_pb_from(pipe, pb_buf, sizeof(pb_buf), &pb_tx_pdos[id], name);
                if ( pb_tx_pdos[id].type() == iit::advr::Ec_slave_pdo::CLIENT_PIPE ) {
                    auto pb_client_pdo = pb_tx_pdos[id].mutable_client_pdo();
                    if ( pb_client_pdo->type() == iit::advr::Client_pipe::CONNECT ) {
                        if ( ! wr_iddp_connected[id] ) { 
                            wr_iddp[id].connect( wr_motor_pdo[id]);
                            wr_iddp_connected[id] = true;
                            DPRINTF ("connect to %s_tx_pdo port\n", wr_motor_pdo[id].c_str() );
                        } 
                        else{
                            DPRINTF (" Already connected to %s_tx_pdo\n", wr_motor_pdo[id].c_str() );                                
                        }
                    } 
                    else if ( pb_client_pdo->type() == iit::advr::Client_pipe::QUIT ) {
                        if ( wr_iddp_connected[id] ) {
                            wr_iddp[id].close_pipe();
                            wr_iddp_connected[id] = false;
                            DPRINTF ("quit %s_tx_pdo\n", wr_motor_pdo[id].c_str() );
                        } 
                        else {
                            DPRINTF (" Already disconnected to %s_tx_pdo\n", wr_motor_pdo[id].c_str() );
                        }               
                    }
                } 
                else {
                    pbDeserialize(id);                             
                }
            } while (nbytes > 0);
            // read pdo
        }     
        
        for ( auto& [id, pipe] : wr_iddp ) {
            if (wr_iddp_connected[id] ){
                pbSerialize(id);
                write_pb_to(pipe, pb_buf, sizeof(pb_buf), &pb_rx_pdos[id], name);
            }
        }
        
    }
    
};

#endif
