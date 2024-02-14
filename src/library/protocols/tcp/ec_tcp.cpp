#include <cassert>
#include <tuple>

#include "protocols/tcp/ec_tcp.h"


EcTCP::EcTCP(std::string host_address,uint32_t host_port):
  EcZmqCmd("tcp",host_address,host_port),
  EcPdo<EcZmqPdo>("tcp",host_address,host_port)
{
}

EcTCP::~EcTCP()
{
    iit::ecat::print_stat ( s_loop );

    stop_logging();
    
    stop();
    
    if(_thread_jointable){
        join();
    }
}

//******************************* INIT *****************************************************//

void EcTCP::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow = tPre = start_time;
    loop_cnt = 0;
}

void EcTCP::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms,_logging);
}

void EcTCP::start_client(uint32_t period_ms,bool logging)
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
            create(false); // non-real time thread
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

void EcTCP::stop_client()
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

void EcTCP::th_loop( void * )
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
    
    // send motors and others pdo
    send_pdo();
}

//******************************* Periodic Activity *****************************************************//
