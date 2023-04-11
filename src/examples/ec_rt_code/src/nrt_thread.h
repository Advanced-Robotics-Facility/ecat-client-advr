#ifndef __UI_THREAD__
#define __UI_THREAD__

#include <utils.h>
#include <pb_utils.h>
#include <thread_util.h>

#include "ec_client_cmd.h"
using slave_descr_t = std::vector<std::tuple<int, int, int>>;

////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////
class UI_thread : public Thread_hook {

private:
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    
    int                 wr_xddp, rd_xddp;
    std::string         rd_pipe_path, wr_pipe_path;
    
    iit::advr::Repl_cmd pb_repl_cmd;
    const slave_descr_t slave_descr;
    uint8_t             pb_buf[MAX_PB_SIZE];    
    
    uint32_t pbSet(uint32_t idx);
    
public:

    UI_thread(std::string _name,
              const slave_descr_t &slave_descr,
              std::string rd,
              std::string wr);

    ~UI_thread();
    virtual void th_init ( void * );
    virtual void th_loop ( void * );

};


#endif
