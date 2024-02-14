#include <cassert>
#include <tuple>

#include "protocols/ipc/iddp/ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcZmqCmd("tcp",host_address,host_port),
  EcPdo<EcPipePdo>("None")
{        
    
}

EcIDDP::~EcIDDP()
{
    iit::ecat::print_stat ( s_loop );
    
    stop_logging();
    
    stop();
    
    if(_thread_jointable){
        join();
    }
}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow = tPre = start_time;
    loop_cnt = 0;
}

void EcIDDP::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms,_logging);
}

void EcIDDP::start_client(uint32_t period_ms,bool logging)
{
    // periodic
    struct timespec ts;
    iit::ecat::us2ts(&ts, 1000*period_ms);
    // period.period is a timeval ... tv_usec 
    period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
//     #ifdef __PREEMPT_RT_
//         schedpolicy = SCHED_FIFO;
//     #else
        schedpolicy = SCHED_OTHER;
//     #endif
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    _logging=logging;

    SSI slave_info;
    retrieve_slaves_info(slave_info);
    
    bool create_thread=false;
    _thread_jointable=create_thread;
    
    if(!slave_info.empty()){
        create_thread=true;
    }
    else{
#ifdef TEST_LIBRARY 
        create_thread=true;
#endif        
    }
    
    if(create_thread){
        try{
            esc_factory(slave_info);
            create(true); // real time thread
            _thread_jointable=true;
            _client_alive=true;
            if(_logging){
                _ec_logger->init_mat_logger(slave_info);
                start_logging();
            }
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }
}

void EcIDDP::stop_client()
{
    stop_logging();
    
    stop();
    
    if(_thread_jointable){
        join();
        _thread_jointable=false;
    }
    
    _client_alive=false;
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;

    if(!_client_alive)
    {
        stop_client();
        return;
    }
    
    // read motors, imu, ft, power board and others pdo information
    read_pdo();

    // write motors and others pdo
    write_pdo();
}
//******************************* Periodic Activity *****************************************************//
