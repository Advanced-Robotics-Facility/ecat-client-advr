#include <cassert>
#include <tuple>

#include "protocol/ipc/zipc/ec_zipc.h"


EcZipc::EcZipc(std::string host_address,uint32_t host_port):
  EcZmqCmd("ipc",host_address,host_port),
  EcPdo<EcZmqPdo>("ipc",host_address,host_port)
{
    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

EcZipc::~EcZipc()
{
    iit::ecat::print_stat ( s_loop );
    
    stop_client();
    
    _client_alive=false;
}

//******************************* INIT *****************************************************//

void EcZipc::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow = tPre = start_time;
    loop_cnt = 0;

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
    }
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
    
    _logging=logging;

    SSI slave_info;
    if(retrieve_slaves_info(slave_info)){
        try{
            esc_factory(slave_info);
            create(false); // non-real time thread
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }
    
}

void EcZipc::stop_client()
{        
    stop();
    
    join();

    stop_logging();
    
    //_client_alive=false;
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
    
    // read motors, imu, ft, power board and others pdo information
    read_pdo();
    
    // send motors and others pdo
    send_pdo();
}

//******************************* Periodic Activity *****************************************************//
