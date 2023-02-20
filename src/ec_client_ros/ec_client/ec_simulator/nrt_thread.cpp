#include "nrt_thread.h"

    
////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////

UI_thread::UI_thread(std::string _name,
                     const slave_descr_t &slave_descr,
                     std::string rd,
                     std::string wr) :          
    slave_descr(slave_descr),
    rd_pipe_path(pipe_prefix + "xddp/" + rd),
    wr_pipe_path(pipe_prefix + "xddp/" + wr)
{
    name = _name;
    // non periodic
    period.period = {0,1};
    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

UI_thread::~UI_thread()
{
    close(wr_xddp);
    close(rd_xddp);
    iit::ecat::print_stat ( s_loop );
}

/*
 * Use protobuf msg iit::advr::Repl_cmd
 * - set type ad iit::advr::CmdType::CTRL_CMD
 * - set header
 * 
 */
uint32_t UI_thread::pbSet(uint32_t idx)
{
    pb_repl_cmd.set_type(iit::advr::CmdType::CTRL_CMD);
    set_pbHeader(pb_repl_cmd.mutable_header(), name, idx);
    return pb_repl_cmd.ByteSize();        
}
    
void UI_thread::th_init ( void * )
{
    pthread_barrier_wait(&threads_barrier);    

    rd_xddp = open ( rd_pipe_path.c_str(), O_RDONLY );
    wr_xddp = open ( wr_pipe_path.c_str(), O_WRONLY | O_NONBLOCK );
    
    if ( rd_xddp <= 0 ) {
        DPRINTF("%s fail open %s\n", __FUNCTION__, rd_pipe_path.c_str());
    }
    if ( wr_xddp <= 0 ) {
        DPRINTF("%s fail open %s\n", __FUNCTION__, wr_pipe_path.c_str());
    }
    if ( rd_xddp <= 0 || wr_xddp <= 0 ) {
        assert (0);
    }
    
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
}

void UI_thread::th_loop ( void * )
{
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    int32_t nbytes_wr, nbytes_rd;
    ///////////////////////////////////////////////////////////////////////
    // 1 : write to RT
    pbSet(loop_cnt);
    nbytes_wr = write_pb_to_RT(rd_xddp, pb_buf, sizeof(pb_buf), &pb_repl_cmd, name);
    ///////////////////////////////////////////////////////////////////////
    // 4 : read from RT -- BLOCKING
    nbytes_rd = read_pb_from_RT(wr_xddp, pb_buf, sizeof(pb_buf), &pb_repl_cmd, name);
    
    if ( nbytes_rd > 0 && nbytes_wr > 0 ) {
        DPRINTF("_-_-_-_-_-_-_-_-_-_-_-_-_-_-_\n");
    }
}


