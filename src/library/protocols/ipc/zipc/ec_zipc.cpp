#include <cassert>
#include <tuple>

#include "protocols/ipc/zipc/ec_zipc.h"


EcZipc::EcZipc(std::string host_address,uint32_t host_port):
  EcCmd("ipc",host_address,host_port),
  EcPdo<EcZmqPdo>("ipc",host_address,host_port)
{
}

EcZipc::~EcZipc()
{
    iit::ecat::print_stat ( s_loop );
    
    stop_logging();
    
    stop();
    
    if(_create_thread){
        join();
    }
}

//******************************* INIT *****************************************************//

void EcZipc::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow = tPre = start_time;
    loop_cnt = 0;
}

void EcZipc::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms,_logging);
}

void EcZipc::start_client(uint32_t period_ms,bool logging)
{
    // periodic
    struct timespec ts;
    iit::ecat::us2ts(&ts, 1000*period_ms);
    // period.period is a timeval ... tv_usec 
    period.period = { ts.tv_sec, ts.tv_nsec / 1000 };

    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    _logging=logging;
    
    SSI slave_info;
    retrieve_slaves_info(slave_info);
    _create_thread=false;
    if(!slave_info.empty()){
        _create_thread=true;
    }
    else{
#ifdef TEST_LIBRARY 
        _create_thread=true;
#endif        
    }
    
    if(_create_thread){
        esc_factory(slave_info);
        create(false); // non-real time thread
        _client_alive=true;
        if(_logging){
            start_logging();
        }
    }
    
}

void EcZipc::stop_client()
{    
    stop_logging();
    
    stop();
    
    join();
    
    _client_alive=false;
    _create_thread=false;
}


//******************************* Periodic Activity *****************************************************//

void EcZipc::th_loop( void * )
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
    
    // Receive motors, imu, ft, power board and others pdo information
    read_pdo();
    
    // Send motors references
    feed_motors();
}

//******************************* Periodic Activity *****************************************************//
