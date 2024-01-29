#include "ec_iface.h"

EcIface::EcIface()
{
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();
    
    pthread_mutex_init(&_mutex_motor_status, NULL);
    pthread_mutex_init(&_mutex_motor_reference, NULL);
    
    pthread_mutex_init(&_mutex_ft6_status, NULL);
    
    pthread_mutex_init(&_mutex_pow_status, NULL);
    
    pthread_mutex_init(&_mutex_imu_status, NULL);
}

EcIface::~EcIface()
{
    pthread_mutex_destroy(&_mutex_motor_status);
    pthread_mutex_destroy(&_mutex_motor_reference);
    
    pthread_mutex_destroy(&_mutex_ft6_status);
    
    pthread_mutex_destroy(&_mutex_pow_status);
    
    pthread_mutex_destroy(&_mutex_imu_status);
}

bool EcIface::is_client_alive()
{
    return _client_alive;
}

void EcIface::start_logging()
{
    stop_logging();
    _ec_logger->start_mat_logger();
}

void EcIface::stop_logging()
{
    _ec_logger->stop_mat_logger();
}

MotorStatusMap EcIface::get_motors_status()
{
    pthread_mutex_lock(&_mutex_motor_status);
    
    auto ret_motor_status_map= _motor_status_map;

    pthread_mutex_unlock(&_mutex_motor_status);
    
    return ret_motor_status_map;
}

void EcIface::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
{
    pthread_mutex_lock(&_mutex_motor_reference);

    _motor_ref_flags = motor_ref_flags;
    _motors_references = motors_references;
    _ec_logger->log_set_motors_ref(_motors_references);

    pthread_mutex_unlock(&_mutex_motor_reference);
}

FtStatusMap EcIface::get_ft6_status()
{
    pthread_mutex_lock(&_mutex_ft6_status);
    
    auto ret_ft_status_map= _ft_status_map;
    
    pthread_mutex_unlock(&_mutex_ft6_status);
    
    return ret_ft_status_map; 
}

PwrStatusMap EcIface::get_pow_status()
{
    pthread_mutex_lock(&_mutex_pow_status);
    
    auto ret_pow_status_map= _pow_status_map;
    
     pthread_mutex_unlock(&_mutex_pow_status);
    
    return ret_pow_status_map; 
}


ImuStatusMap EcIface::get_imu_status()
{
    pthread_mutex_lock(&_mutex_imu_status);
    
    auto ret_imu_status_map= _imu_status_map;
    
    pthread_mutex_unlock(&_mutex_imu_status);
    
    return ret_imu_status_map; 
}
    
