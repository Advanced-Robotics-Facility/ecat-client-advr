#include <cassert>
#include <tuple>

#include "protocol/ipc/iddp/ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcZmqCmd("tcp",host_address,host_port),
  EcPdo<EcPipePdo>("NoNe")
{        
    
#ifdef PREEMPT_RT
    schedpolicy = SCHED_FIFO;
    priority = sched_get_priority_max (99);
#else
    #ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
    #else
        schedpolicy = SCHED_OTHER;
    #endif
        priority = sched_get_priority_max ( schedpolicy ) / 2;
#endif
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

EcIDDP::~EcIDDP()
{
    iit::ecat::print_stat ( s_loop );
    
    stop_client();
    
    _client_alive=false;
}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    if(!init_read_pdo()){
        DPRINTF("Client thread not initialized!\n");
        _client_alive=false;
        stop_client();
    }
    else{
        if(_logging){
            start_logging();
        }
        DPRINTF("Client thread initialized!\n");
        sync_client_thread();
        start_time = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
	    tNow = tPre = start_time;
	    loop_cnt = 0;
    }
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
    period.period = {0,1}; 
    _period_ns = 1000000*period_ms;

    _logging=logging;

    SSI slave_info;
    if(retrieve_slaves_info(slave_info)){
        try{
            esc_factory(slave_info);
            create(true); // real time thread
            sync_client_thread();
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }
}

void EcIDDP::stop_client()
{
    stop();
    
    join();
    
    stop_logging();

    //_client_alive=false;
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
    s_loop ( tNow - tPre );
    float time_elapsed_ms= (static_cast<float>((tNow-tPre))/1000000);
    DPRINTF("IDDP thread sample time %f\n",time_elapsed_ms);
    tPre = tNow;
    
    loop_cnt++;

    if(!_client_alive){
        stop_client();
        return;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += _period_ns;

    pthread_mutex_lock(&_mutex_update);
    if(update_count==0){
       pthread_cond_timedwait(&_update_cond, &_mutex_update, &ts);
    }
    update_count--;
    update_count=std::max(update_count,0);
    pthread_mutex_unlock(&_mutex_update);
    
    // read motors, imu, ft, power board and others pdo information
    read_pdo();

    // write motors and others pdo
    write_pdo();

}
//******************************* Periodic Activity *****************************************************//
