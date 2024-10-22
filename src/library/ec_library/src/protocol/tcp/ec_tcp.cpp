#include <cassert>
#include <tuple>

#include "protocol/tcp/ec_tcp.h"


EcTCP::EcTCP(std::string host_address,uint32_t host_port):
  EcZmqCmd("tcp",host_address,host_port),
  EcPdo<EcZmqPdo>("tcp",host_address,host_port)
{
    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    // non-periodic
    period.period = {0,1}; 
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    _client_thread_info.priority=priority;
    _client_thread_info.policy=schedpolicy;
}

EcTCP::~EcTCP()
{
    iit::ecat::print_stat ( s_loop );
    
    stop_client();
}

//******************************* INIT *****************************************************//

void EcTCP::th_init ( void * )
{
    if(!init_read_pdo()){
        DPRINTF("Client thread not initialized!\n");
        _client_status.run_loop=false;
        stop_client();
    }
    else{
        _client_thread_info.cpu=sched_getcpu();
        DPRINTF("Client thread initialized, ");
        DPRINTF("id: %ld cpu: %d, priority %d\n",pthread_self(),_client_thread_info.cpu,_client_thread_info.priority);
        _client_status.run_loop=true;
        sync_client_thread();
        start_time = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
	    tNow = tPre = start_time;
	    loop_cnt = 0;
    }
}

void EcTCP::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms);
}

void EcTCP::start_client(uint32_t period_ms)
{
    _period_ns = 1000000*period_ms;

    SSI slave_info;
    if(retrieve_slaves_info(slave_info)){
        try{
            esc_factory(slave_info);
            create(false); // non-real time thread
            sync_client_thread();
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }
}

void EcTCP::stop_client()
{
    stop();
    
    join();

    _client_status.run_loop=_run_loop;
}


//******************************* Periodic Activity *****************************************************//

void EcTCP::th_loop( void * )
{
    tNow = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
    s_loop ( tNow - tPre );
    float sample_elapsed_ms= (static_cast<float>((tNow-tPre))/1000000);
    //DPRINTF("IDDP thread sample time %f\n",sample_elapsed_ms);
    tPre = tNow;
    
    loop_cnt++;

    if(_client_status.status==ClientStatusEnum::NOT_ALIVE){
        _run_loop = false;
        _client_status.run_loop=_run_loop;
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
                _client_status.run_loop=_run_loop;
                return;
            }
       }
    }
    _update_count--;
    _update_count=std::max(_update_count,0);
    pthread_mutex_unlock(&_mutex_update);
    
    // read motors, imu, ft, power board and others pdo information
    read_pdo();

    // send motors and others pdo
    send_pdo();
}

//******************************* Periodic Activity *****************************************************//
