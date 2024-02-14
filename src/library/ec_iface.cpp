#include "ec_iface.h"

EcIface::EcIface()
{
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();
    
    pthread_mutex_init(&_mutex_motor_status, NULL);
    pthread_mutex_init(&_mutex_motor_reference, NULL);
    
    pthread_mutex_init(&_mutex_ft_status, NULL);
    
    pthread_mutex_init(&_mutex_pow_status, NULL);
    
    pthread_mutex_init(&_mutex_imu_status, NULL);
    
    pthread_mutex_init(&_mutex_valve_status, NULL);
    pthread_mutex_init(&_mutex_valve_reference, NULL);
    
    
    pthread_mutex_init(&_mutex_pump_status, NULL);
    pthread_mutex_init(&_mutex_pump_reference, NULL);
    
    _consoleLog=spdlog::get("console");
    if(!_consoleLog)
    {
        createLogger("console","client");
        _consoleLog=spdlog::get("console");
    }
    _consoleLog->info("EtherCAT Client initialized");
}

EcIface::~EcIface()
{
    pthread_mutex_destroy(&_mutex_motor_status);
    pthread_mutex_destroy(&_mutex_motor_reference);
    
    pthread_mutex_destroy(&_mutex_ft_status);
    
    pthread_mutex_destroy(&_mutex_pow_status);
    
    pthread_mutex_destroy(&_mutex_imu_status);
    
    pthread_mutex_destroy(&_mutex_valve_status);
    pthread_mutex_destroy(&_mutex_valve_reference);
    
    pthread_mutex_destroy(&_mutex_pump_status);
    pthread_mutex_destroy(&_mutex_pump_reference);
    
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

void EcIface::get_motors_status(MotorStatusMap &motor_status_map)
{
    pthread_mutex_lock(&_mutex_motor_status);
    motor_status_map= _motor_status_map;
    pthread_mutex_unlock(&_mutex_motor_status);
}

void EcIface::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
{
    pthread_mutex_lock(&_mutex_motor_reference);

    _motor_ref_flags = motor_ref_flags;
    _motors_references = motors_references;
    _ec_logger->log_set_motors_ref(_motors_references);

    pthread_mutex_unlock(&_mutex_motor_reference);
}

void EcIface::get_ft_status(FtStatusMap &ft_status_map)
{
    pthread_mutex_lock(&_mutex_ft_status);
    ft_status_map= _ft_status_map;
    pthread_mutex_unlock(&_mutex_ft_status);
}

void EcIface::get_pow_status(PwrStatusMap &pow_status_map)
{
    pthread_mutex_lock(&_mutex_pow_status);
    pow_status_map= _pow_status_map;
    pthread_mutex_unlock(&_mutex_pow_status);
}


void EcIface::get_imu_status(ImuStatusMap &imu_status_map)
{
    pthread_mutex_lock(&_mutex_imu_status);
    imu_status_map= _imu_status_map;
    pthread_mutex_unlock(&_mutex_imu_status);
}

void EcIface::get_valve_status(ValveStatusMap &valve_status_map)
{
    pthread_mutex_lock(&_mutex_valve_status);
    valve_status_map= _valve_status_map;
    pthread_mutex_unlock(&_mutex_valve_status);
}

void EcIface::set_valve_references()
{
    pthread_mutex_lock(&_mutex_valve_reference);

    
    pthread_mutex_unlock(&_mutex_valve_reference);
}

void EcIface::get_pump_status(PumpStatusMap &pump_status_map)
{
    pthread_mutex_lock(&_mutex_pump_status);
    pump_status_map= _pump_status_map;
    pthread_mutex_unlock(&_mutex_pump_status);
}

void EcIface::set_pump_references()
{
    pthread_mutex_lock(&_mutex_pump_reference);

    
    pthread_mutex_unlock(&_mutex_pump_reference);
}
 
bool EcIface::pdo_aux_cmd_sts(const PAC & pac)
{    
    for( const auto &[esc_id,pdo_aux_cmd] : pac)
    {
        if(_motor_status_map.count(esc_id) > 0)
        {
            uint32_t cmd_aux_sts=std::get<11>(_motor_status_map[esc_id]);
            

            uint32_t brake_sts = cmd_aux_sts & 3; //00 unknown
                                                    //01 release brake 
                                                    //10 enganged brake
                                                    //11 error
                                                    
            uint32_t led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
            
            
            switch(pdo_aux_cmd){
                
                case to_underlying_enum(PdoAuxCmdTypeEnum::BRAKE_RELEASE):{
                    if(brake_sts!=1){ 
                        //_consoleLog->error("esc_id: {}, brake status: {} ---> brake requested: {} ",esc_id, brake_sts, PdoAuxCmdTypeEnum::BRAKE_RELEASE);
                        return false;
                    }
                }break;
                case to_underlying_enum(PdoAuxCmdTypeEnum::BRAKE_ENGAGE):{
                    if(brake_sts!=2){ 
                        //_consoleLog->error("esc_id: {}, brake status: {} ---> brake requested: {} ",esc_id, brake_sts, PdoAuxCmdTypeEnum::BRAKE_ENGAGE);
                        return false;
                    }
                }break;
                case to_underlying_enum(PdoAuxCmdTypeEnum::LED_ON):{
                    if(led_sts!=1){ 
                        //_consoleLog->error("esc_id: {}, led status: {} ---> led requested: {} ",esc_id, led_sts, PdoAuxCmdTypeEnum::LED_ON);
                        return false;
                    }
                }break;
                case to_underlying_enum(PdoAuxCmdTypeEnum::LED_OFF):{
                    if(led_sts!=0){ 
                        //_consoleLog->error("esc_id: {}, led status: {} ---> led requested: {} ",esc_id, led_sts, PdoAuxCmdTypeEnum::LED_OFF);
                        return false;
                    }
                }break;
            }
        }
        else
        {
            //_consoleLog->error("esc_id: {}, doesn't exist, please restart the request", esc_id);
            return false; // return false if the esc id it's not present into the motor status map.
        }
    }
        
    return true;
} 
