#include <cassert>
#include <tuple>

#include "ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcCmd(host_address,host_port)
{
    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
     _mutex_motor_status= std::make_shared<std::mutex>();
    _mutex_motor_reference= std::make_shared<std::mutex>();

    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();

    _mutex_ft6_status= std::make_shared<std::mutex>();
    
    _mutex_pow_status= std::make_shared<std::mutex>();
    
    _mutex_imu_status= std::make_shared<std::mutex>();
}

EcIDDP::~EcIDDP()
{

}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    
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
    iit::ecat::us2ts(&ts, period_ms);
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
    
    create(true); // real time thread
    _client_alive=true;
    
}

void EcIDDP::stop_client()
{
    stop();
    
    stop_logging();
}

bool EcIDDP::is_client_alive()
{
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
    // Receive motors, imu, ft, power board pdo information // 

    // Send motors references
    _mutex_motor_reference->lock();

    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE &&
        !_motors_references.empty())
    {
        if(_motor_ref_flags==MotorRefFlags::FLAG_MULTI_REF )
        {
            // SEND TCP
        }
        else
        {
           // SEND TCP
        }
    }

    _mutex_motor_reference->unlock();
}

void EcIDDP::periodicActivity()
{
    
        
}
//******************************* Periodic Activity *****************************************************//



void EcIDDP::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
{
    _mutex_motor_reference->lock();

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
                //_consoleLog->error("Motors references vector is empty!, please fill the vector");
           }
       }
       else
       {
           if(motor_ref_flags!=MotorRefFlags::FLAG_NONE)
           {
                //_consoleLog->error("Wrong motors references flag!");
           }
       }
    }
    else
    {
        //_consoleLog->error("UDP client not alive, please stop the main process");
    }

    _mutex_motor_reference->unlock();
}

MotorStatusMap EcIDDP::get_motors_status()
{
    _mutex_motor_status->lock();
    
    auto ret_motor_status_map= _motor_status_map;
    
    _mutex_motor_status->unlock();
    
    return ret_motor_status_map;
}

FtStatusMap EcIDDP::get_ft6_status()
{
    _mutex_ft6_status->lock();
    
    auto ret_ft_status_map= _ft_status_map;
    
    _mutex_ft6_status->unlock();
    
    return ret_ft_status_map; 
}

PwrStatusMap EcIDDP::get_pow_status()
{
    _mutex_pow_status->lock();
    
    auto ret_pow_status_map= _pow_status_map;
    
    _mutex_pow_status->unlock();
    
    return ret_pow_status_map; 
}


ImuStatusMap EcIDDP::get_imu_status()
{
    _mutex_imu_status->lock();
    
    auto ret_imu_status_map= _imu_status_map;
    
    _mutex_imu_status->unlock();
    
    return _imu_status_map; 
}
bool EcIDDP::pdo_aux_cmd_sts(const PAC & pac)
{
    return false;
}
