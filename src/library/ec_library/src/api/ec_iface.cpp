#include "api/ec_iface.h"

EcIface::EcIface()
{
    _client_status.status=ClientStatusEnum::IDLE;
    _client_status.run_loop=false;

    _client_thread_info.cpu=-1;
    _client_thread_info.priority=0;
    _client_thread_info.policy=SCHED_OTHER;
    
    pthread_mutexattr_t mutex_update_attr;

    pthread_mutexattr_init(&mutex_update_attr);
    pthread_mutexattr_settype(&mutex_update_attr, PTHREAD_MUTEX_NORMAL);
    pthread_mutexattr_setpshared(&mutex_update_attr, PTHREAD_PROCESS_PRIVATE);
    pthread_mutexattr_setprotocol(&mutex_update_attr, PTHREAD_PRIO_NONE);
    
    int ret=pthread_mutex_init(&_mutex_client_thread, &mutex_update_attr);
    pthread_mutexattr_destroy(&mutex_update_attr);
    if (ret != 0){
        throw std::runtime_error("fatal error: cannot initialize mutex_client_thread, reason: "+std::to_string(ret));
    }

    pthread_condattr_t update_attr;
    pthread_condattr_init(&update_attr);
    pthread_condattr_setpshared(&update_attr, PTHREAD_PROCESS_PRIVATE);
    pthread_condattr_setclock(&update_attr, CLOCK_MONOTONIC);

    ret=pthread_cond_init(&_client_thread_cond,&update_attr);
    pthread_condattr_destroy(&update_attr);
    if (ret != 0){
        throw std::runtime_error("fatal error: cannot initialize client_thread_cond, reason: "+std::to_string(ret));
    }
    
    _consoleLog=spdlog::get("console");
    if(!_consoleLog){
        createLogger("console","client");
        _consoleLog=spdlog::get("console");
    }
    _write_device={false,false,false};
    
    _consoleLog->info("EtherCAT Client initialized");
}

EcIface::~EcIface()
{
    pthread_mutex_destroy(&_mutex_client_thread);
    pthread_cond_destroy(&_client_thread_cond);
    
    _consoleLog->info("EtherCAT Client closed");
    _consoleLog.reset();
}

EcIface::CLIENT_STATUS EcIface::get_client_status()
{
    return _client_status;
}

EcIface::CLIENT_THREAD_INFO EcIface::get_client_thread_info()
{
    return _client_thread_info;
}

void EcIface::set_slaves_info(SSI slave_info)
{
    _fake_slave_info=slave_info;
}

void EcIface::read()
{
    //note: only one thread is allowed to pop data
    while(_motor_status_queue.pop(_motor_status_map))
    {}

    while(_pow_status_queue.pop(_pow_status_map))
    {}

    while(_ft_status_queue.pop(_ft_status_map))
    {}

    while(_imu_status_queue.pop(_imu_status_map))
    {}

    while(_valve_status_queue.pop(_valve_status_map))
    {}

    while(_pump_status_queue.pop(_pump_status_map))
    {}
}

void EcIface::get_motor_status(MotorStatusMap &motor_status_map)
{
    motor_status_map= _motor_status_map;
}

void EcIface::set_reference_flag(uint32_t reference_flag)
{
    _reference_flag=reference_flag;
}

void EcIface::set_motor_reference(const MotorReferenceMap motor_reference_map)
{
    if(check_maps(_motor_reference_map,motor_reference_map,"motor")){
        _motor_reference_map = motor_reference_map;
        _write_device[DeviceCtrlType::MOTOR]=true;
    }
}

void EcIface::get_ft_status(FtStatusMap &ft_status_map)
{
    ft_status_map= _ft_status_map;
}

void EcIface::get_pow_status(PwrStatusMap &pow_status_map)
{
    pow_status_map= _pow_status_map;
}


void EcIface::get_imu_status(ImuStatusMap &imu_status_map)
{
    imu_status_map= _imu_status_map;
}

void EcIface::get_valve_status(ValveStatusMap &valve_status_map)
{
    valve_status_map= _valve_status_map;
}

void EcIface::set_valve_reference(const ValveReferenceMap valve_reference_map)
{
    if(check_maps(_valve_reference_map,valve_reference_map,"valve")){
        _valve_reference_map=valve_reference_map;
        _write_device[DeviceCtrlType::VALVE]=true;
    }
}

void EcIface::get_pump_status(PumpStatusMap &pump_status_map)
{
    pump_status_map= _pump_status_map;
}

void EcIface::set_pump_reference(const PumpReferenceMap pump_reference_map)
{
    if(check_maps(_pump_reference_map,pump_reference_map,"pump")){
        _pump_reference_map=pump_reference_map;
        _write_device[DeviceCtrlType::PUMP]=true;
    }
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

void EcIface::sync_client_thread(void) {
    
    pthread_mutex_lock(&_mutex_client_thread);

    if(_waiting_client_counter<2){
        _waiting_client_counter++;
    }

    if (_waiting_client_counter == 2) {
        pthread_cond_broadcast(&_client_thread_cond);
        _waiting_client_counter=0;
    } else {
        pthread_cond_wait(&_client_thread_cond, &_mutex_client_thread);
    }

    pthread_mutex_unlock(&_mutex_client_thread);
}

template <typename T>
bool EcIface::check_maps(const std::map<int32_t,T>& map1,const std::map<int32_t,T>& map2,std::string map_type)
{
    if(map1.size()==0){
        DPRINTF("No %s detected!\n",map_type.c_str());
        return false;
    }
    else if(map2.size()==0){
        DPRINTF("Got an empy %s references map\n",map_type.c_str());
        return false;
    }
    else if(map1.size()!=map2.size()){
        DPRINTF("Got an different %s references size\n",map_type.c_str());
        return false;
    }
    else{
        for ( const auto &[esc_id,tx_pdo] : map2) {
            if(map1.count(esc_id)==0){
                DPRINTF("Esc id [%d] is not a %s\n",esc_id,map_type.c_str());
                return false; 
            }
        }
    }
    return true;
}
