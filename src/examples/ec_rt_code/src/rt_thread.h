#ifndef __RT_THREAD__
#define __RT_THREAD__

#include <thread_util.h>
#include <pb_utils.h>
#include <esc_pipe_iface.h>

using slave_descr_t = std::vector<std::tuple<int, int, int>>;

enum FSM_type : uint32_t {

    INIT        = 0x0,
    IDLE,
    HOMING,
    MOVING,
    SINE,
    STOP,
};

////////////////////////////////////////////////////
// 
// 
//
////////////////////////////////////////////////////
class RT_motor_thread : public Thread_hook {

private:
    
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    
    std::string         rd_pipe, wr_pipe;
    XDDP_pipe           rd_xddp, wr_xddp;
    
    iit::advr::Repl_cmd             pb_repl_cmd;
    
    std::map<int, SH_PIFACE>  escs_iface;
    
    slave_descr_t slave_descr;
    uint8_t  pb_buf[MAX_PB_SIZE];

    uint32_t pbSet(uint32_t idx);
       
    std::string rt_thread_start;
    std::string _zmq_uri;
    int _timeout_ms;
    
public:

    RT_motor_thread(std::string _name,
                    std::string rt_thread_start,
                    std::string rd,
                    std::string wr,
                    uint32_t th_period_us );
    
    ~RT_motor_thread();

    virtual void th_init ( void * );    
    virtual void th_loop ( void * );

    
};

#endif
