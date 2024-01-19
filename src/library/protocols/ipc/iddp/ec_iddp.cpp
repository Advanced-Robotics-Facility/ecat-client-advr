#include <cassert>
#include <tuple>

#include "protocols/ipc/iddp/ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcCmd("tcp",host_address,host_port)
{    
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();
}

EcIDDP::~EcIDDP()
{
    iit::ecat::print_stat ( s_loop );
    
    pthread_mutex_destroy(&_mutex_motor_status);
    pthread_mutex_destroy(&_mutex_motor_reference);
    
    pthread_mutex_destroy(&_mutex_ft6_status);
    
    pthread_mutex_destroy(&_mutex_pow_status);
    
    pthread_mutex_destroy(&_mutex_imu_status);
    
    stop();
    
    join();
}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
    
    std::string robot_name = "NoNe";
    _escs_factory= std::make_shared<EscFactory>(_slave_info,robot_name);
    
    pthread_mutex_init(&_mutex_motor_status, NULL);
    pthread_mutex_init(&_mutex_motor_reference, NULL);
    
    pthread_mutex_init(&_mutex_ft6_status, NULL);
    
    pthread_mutex_init(&_mutex_pow_status, NULL);
    
    pthread_mutex_init(&_mutex_imu_status, NULL);
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

bool EcIDDP::is_client_alive()
{
    _client_alive = client_sts();
    return _client_alive;
}

void EcIDDP::start_logging()
{
    stop_logging();
    _ec_logger->start_mat_logger();
}

void EcIDDP::stop_logging()
{
    _ec_logger->stop_mat_logger();
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    if(!client_sts())
    {
        stop_client();
        return;
    }
    
    // Receive motors, imu, ft, power board pdo information // 
    pthread_mutex_lock(&_mutex_motor_status);
    _escs_factory->read_motors(_motor_status_map);
    _ec_logger->log_motors_sts(_motor_status_map);
    pthread_mutex_unlock(&_mutex_motor_status);
    
    pthread_mutex_lock(&_mutex_ft6_status);
    _escs_factory->read_fts(_ft_status_map);
    _ec_logger->log_ft6_sts(_ft_status_map);
    pthread_mutex_unlock(&_mutex_ft6_status);
    
    pthread_mutex_lock(&_mutex_imu_status);
    _escs_factory->read_imus(_imu_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
    pthread_mutex_unlock(&_mutex_imu_status);

    
    pthread_mutex_lock(&_mutex_pow_status);
    _escs_factory->read_pows(_pow_status_map);
    _ec_logger->log_pow_sts(_pow_status_map);
    pthread_mutex_unlock(&_mutex_pow_status);
    
    // Send motors references
    pthread_mutex_lock(&_mutex_motor_reference);

    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE &&
       !_motors_references.empty())
    {
        _escs_factory->feed_motors(_motors_references);
        _ec_logger->log_motors_ref(_motors_references);
    }
    
    pthread_mutex_unlock(&_mutex_motor_reference);
}

void EcIDDP::periodicActivity()
{
    
        
}
//******************************* Periodic Activity *****************************************************//



void EcIDDP::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
{
    pthread_mutex_lock(&_mutex_motor_reference);

    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();

    if(_client_alive)
    {
       if(motor_ref_flags==MotorRefFlags::FLAG_MULTI_REF ||
          motor_ref_flags==MotorRefFlags::FLAG_LAST_REF)
       {
           if(!motors_references.empty())
           {
                _motor_ref_flags = motor_ref_flags;
                _motors_references = motors_references;
                _ec_logger->log_set_motors_ref(_motors_references);
           }
           else
           {
               DPRINTF("Motors references vector is empty!, please fill the vector\n");
           }
       }
       else
       {
           if(motor_ref_flags!=MotorRefFlags::FLAG_NONE)
           {
               DPRINTF("Wrong motors references flag!\n");
           }
       }
    }
    else
    {
        DPRINTF("client not alive, please stop the main process\n");
    }

    pthread_mutex_unlock(&_mutex_motor_reference);
}

MotorStatusMap EcIDDP::get_motors_status()
{
    pthread_mutex_lock(&_mutex_motor_status);
    
    auto ret_motor_status_map= _motor_status_map;

    pthread_mutex_unlock(&_mutex_motor_status);
    
    return ret_motor_status_map;
}

FtStatusMap EcIDDP::get_ft6_status()
{
    pthread_mutex_lock(&_mutex_ft6_status);
    
    auto ret_ft_status_map= _ft_status_map;
    
    pthread_mutex_unlock(&_mutex_ft6_status);
    
    return ret_ft_status_map; 
}

PwrStatusMap EcIDDP::get_pow_status()
{
    pthread_mutex_lock(&_mutex_pow_status);
    
    auto ret_pow_status_map= _pow_status_map;
    
     pthread_mutex_unlock(&_mutex_pow_status);
    
    return ret_pow_status_map; 
}


ImuStatusMap EcIDDP::get_imu_status()
{
    pthread_mutex_lock(&_mutex_imu_status);
    
    auto ret_imu_status_map= _imu_status_map;
    
    pthread_mutex_unlock(&_mutex_imu_status);
    
    return ret_imu_status_map; 
}
bool EcIDDP::pdo_aux_cmd_sts(const PAC & pac)
{
    return false;
}
