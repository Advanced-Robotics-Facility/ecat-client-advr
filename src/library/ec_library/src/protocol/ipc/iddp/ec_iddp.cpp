#include <cassert>
#include <tuple>

#include "protocol/ipc/iddp/ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcZmqCmd("tcp",host_address,host_port),
  EcPdo<EcPipePdo>("NoNe")
{ 
    schedpolicy = SCHED_OTHER;
#if defined(PREEMPT_RT) || defined(__COBALT__)       
    schedpolicy = SCHED_FIFO;
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    // non-periodic
    period.period = {0,1}; 
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
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    tNow = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
    s_loop ( tNow - tPre );
    float sample_elapsed_ms= (static_cast<float>((tNow-tPre))/1000000);
    //DPRINTF("IDDP thread sample time %f\n",sample_elapsed_ms);
    tPre = tNow;
    
    loop_cnt++;

    if(!_client_alive){
        _run_loop = false;
        return;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    iit::ecat::add_timespec(&ts,_period_ns);

    pthread_mutex_lock(&_mutex_update);
    if(_update_count==0){
       int ret = pthread_cond_timedwait(&_update_cond, &_mutex_update, &ts);
       if(ret!=0){
            if(ret!=ETIMEDOUT){
                DPRINTF("Error on pthread_cond_timedwait reason: %d\n",ret);
                _run_loop = false;
                return;
            }
       }
    }
    _update_count--;
    _update_count=std::max(_update_count,0);
    pthread_mutex_unlock(&_mutex_update);
    
    // read motors, imu, ft, power board and others pdo information
    read_pdo();

    // write motors and others pdo
    write_pdo();
}
//******************************* Periodic Activity *****************************************************//
