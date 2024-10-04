#include "api/ec_iface.h"

EcIface::EcIface()
{
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;

    _client_status.status=ClientStatusEnum::IDLE;
    _client_status.run_loop=false;

    _client_thread_info.cpu=-1;
    _client_thread_info.priority=0;
    _client_thread_info.policy=SCHED_OTHER;

    for(uint8_t i=0;i<NUM_DEVICES_CTRL;i++){
        _client_status.devices_started.push_back(false);
    }

    pthread_mutexattr_t mutex_update_attr;

    pthread_mutexattr_init(&mutex_update_attr);
    pthread_mutexattr_settype(&mutex_update_attr, PTHREAD_MUTEX_NORMAL);
    pthread_mutexattr_setpshared(&mutex_update_attr, PTHREAD_PROCESS_PRIVATE);
    pthread_mutexattr_setprotocol(&mutex_update_attr, PTHREAD_PRIO_NONE);
    
    int ret=0;
    bool error=false;
    ret=pthread_mutex_init(&_mutex_update, &mutex_update_attr);
    if (ret != 0){
        pthread_mutexattr_destroy(&mutex_update_attr);
        throw std::runtime_error("fatal error: cannot initialize mutex_update, reason: "+std::to_string(ret));
    }
    ret=pthread_mutex_init(&_mutex_client_thread, &mutex_update_attr);
    pthread_mutexattr_destroy(&mutex_update_attr);
    if (ret != 0){
        throw std::runtime_error("fatal error: cannot initialize mutex_client_thread, reason: "+std::to_string(ret));
    }

    pthread_condattr_t update_attr;
    pthread_condattr_init(&update_attr);
    pthread_condattr_setpshared(&update_attr, PTHREAD_PROCESS_PRIVATE);
    pthread_condattr_setclock(&update_attr, CLOCK_MONOTONIC);

    ret=pthread_cond_init(&_update_cond, &update_attr);
    if (ret != 0){
        pthread_condattr_destroy(&update_attr);
        throw std::runtime_error("fatal error: cannot initialize update_cond, reason: "+std::to_string(ret));
    }
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
    
    _consoleLog->info("EtherCAT Client initialized");
}

EcIface::~EcIface()
{
    pthread_mutex_destroy(&_mutex_update);
    pthread_cond_destroy(&_update_cond);
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

void EcIface::start_logging()
{
    stop_logging();
    _ec_logger->start_mat_logger();
}

void EcIface::stop_logging()
{
    _ec_logger->stop_mat_logger();
}

void EcIface::log()
{
    _ec_logger->log_motors_sts(_motor_status_map);
    _ec_logger->log_pow_sts(_pow_status_map);
    _ec_logger->log_ft_sts(_ft_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
    _ec_logger->log_valve_sts(_valve_status_map);
    _ec_logger->log_pump_sts(_pump_status_map);


    _ec_logger->log_motors_ref(_motors_references); 
    _ec_logger->log_valve_ref(_valves_references);
    _ec_logger->log_pump_ref(_pumps_references);
}

void EcIface::set_slaves_info(SSI slave_info)
{
    _fake_slave_info=slave_info;
}

void EcIface::read()
{
    wake_client_thread();

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
void EcIface::write()
{
    ///note: only one thread is allowed to push data
    _motors_references_queue.push(_motors_references);
    _valves_references_queue.push(_valves_references);
    _pumps_references_queue.push(_pumps_references);

    wake_client_thread();
}

void EcIface::get_motors_status(MotorStatusMap &motor_status_map)
{
    motor_status_map= _motor_status_map;
}

void EcIface::set_motors_references(const MotorReferenceMap motors_references)
{
    if(check_maps(_motors_references,motors_references,"motor")){
        _motors_references = motors_references;
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

void EcIface::set_valves_references(const ValveReferenceMap valves_references)
{
    if(check_maps(_valves_references,valves_references,"valve")){
        _valves_references=valves_references;
    }
}

void EcIface::get_pump_status(PumpStatusMap &pump_status_map)
{
    pump_status_map= _pump_status_map;
}

void EcIface::set_pumps_references(const PumpReferenceMap pumps_references)
{
    if(check_maps(_pumps_references,pumps_references,"pump")){
        _pumps_references=pumps_references;
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

void EcIface::wake_client_thread()
{
    pthread_mutex_lock(&_mutex_update);
    _update_count++;
    _update_count=std::min(_update_count,10);
    pthread_cond_signal(&_update_cond);
    pthread_mutex_unlock(&_mutex_update);

}

bool EcIface::all_devices_stopped(){
    for(const auto& device_started:_client_status.devices_started){
        if(device_started){
            return false;
        }
    }
    return true;
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
