#include <cassert>
#include <tuple>

#include "protocols/ipc/iddp/ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcCmd("tcp",host_address,host_port)
{        
    std::string robot_name = "NoNe";
    _ec_pdo= std::make_shared<EcPdo<EcPipePdo>>(robot_name);
}

EcIDDP::~EcIDDP()
{
    iit::ecat::print_stat ( s_loop );
    
    stop();
    
    join();
}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
    
    _ec_pdo->esc_factory(_slave_info);
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
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    _logging=logging;
    if(_logging)
    {
        start_logging();
    }
    
    retrieve_slaves_info(_slave_info);
    
//     if(!_slave_info.empty()){
        create(true); // real time thread
        _client_alive=true;
//     }
    
}

void EcIDDP::stop_client()
{
    stop_logging();
    
    stop();
    
    join();
    
    _client_alive=false;
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    _client_alive=client_sts();
    
    if(!_client_alive)
    {
        stop_client();
        return;
    }
    
    // Receive motors, imu, ft, power board pdo information // 
    pthread_mutex_lock(&_mutex_motor_status);
    _ec_pdo->read_motor_pdo(_motor_status_map);
    _ec_logger->log_motors_sts(_motor_status_map);
    pthread_mutex_unlock(&_mutex_motor_status);
    
    pthread_mutex_lock(&_mutex_ft6_status);
    _ec_pdo->read_ft_pdo(_ft_status_map);
    _ec_logger->log_ft6_sts(_ft_status_map);
    pthread_mutex_unlock(&_mutex_ft6_status);
    
    pthread_mutex_lock(&_mutex_imu_status);
    _ec_pdo->read_imu_pdo(_imu_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
    pthread_mutex_unlock(&_mutex_imu_status);

    
    pthread_mutex_lock(&_mutex_pow_status);
    _ec_pdo->read_pow_pdo(_pow_status_map);
    _ec_logger->log_pow_sts(_pow_status_map);
    pthread_mutex_unlock(&_mutex_pow_status);
    
    // Send motors references
    pthread_mutex_lock(&_mutex_motor_reference);

    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE &&
       !_motors_references.empty())
    {
        _ec_pdo->write_motor_pdo(_motors_references);
        _ec_logger->log_motors_ref(_motors_references);
    }
    
    pthread_mutex_unlock(&_mutex_motor_reference);
}
//******************************* Periodic Activity *****************************************************//

bool EcIDDP::pdo_aux_cmd_sts(const PAC & pac)
{
    return false;
}
