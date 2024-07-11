#include "api/ec_iface.h"

EcIface::EcIface()
{
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    _client_status=ClientStatus::IDLE;
    
    _motor_ref_flags=RefFlags::FLAG_NONE;
    _motors_references.clear();
    
    _valve_ref_flags=RefFlags::FLAG_NONE;
    _valves_references.clear();
    
    _pump_ref_flags=RefFlags::FLAG_NONE;
    _pumps_references.clear();
    
    pthread_mutex_init(&_mutex_read, NULL);
    pthread_cond_init(&read_cond,NULL);

    pthread_mutex_init(&_mutex_write, NULL);
    pthread_cond_init(&write_cond,NULL);
    
    _consoleLog=spdlog::get("console");
    if(!_consoleLog)
    {
        createLogger("console","client");
        _consoleLog=spdlog::get("console");
    }
    
    _client_alive=true;
    
    _consoleLog->info("EtherCAT Client initialized");
}

EcIface::~EcIface()
{
    pthread_mutex_destroy(&_mutex_read);
    pthread_cond_destroy(&read_cond);
    pthread_mutex_destroy(&_mutex_write);
    pthread_cond_destroy(&write_cond);
    _consoleLog->info("EtherCAT Client closed");
    _consoleLog.reset();
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

void EcIface::log()
{
    _ec_logger->log_motors_sts(_motor_status_map);
    _ec_logger->log_ft_sts(_ft_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
    _ec_logger->log_valve_sts(_valve_status_map);
    _ec_logger->log_pump_sts(_pump_status_map);


    _ec_logger->log_motors_ref(_motors_references); 
    _ec_logger->log_valve_ref(_valves_references);
    _ec_logger->log_pump_ref(_pumps_references);
}

void EcIface::test_client(SSI slave_info)
{
    _fake_slave_info=slave_info;
}

bool EcIface::read()
{
    //read
    pthread_mutex_lock(&_mutex_read);
    pthread_cond_wait(&read_cond, &_mutex_read);
    pthread_mutex_unlock(&_mutex_read);

    while(_motor_status_queue.pop(_motor_status_map))
    {}

    while(_ft_status_queue.pop(_ft_status_map))
    {}

    while(_imu_status_queue.pop(_imu_status_map))
    {}

    while(_valve_status_queue.pop(_valve_status_map))
    {}

    while(_pump_status_queue.pop(_pump_status_map))
    {}

    return true;
}
bool EcIface::write()
{
    //write
    _motors_references_queue.push(_motors_references);
    _valves_references_queue.push(_valves_references);
    _pumps_references_queue.push(_pumps_references);

    pthread_mutex_lock(&_mutex_write);
    pthread_cond_wait(&write_cond, &_mutex_write);
    pthread_mutex_unlock(&_mutex_write);

    return true;
}

void EcIface::sync_read(void) {
    
    pthread_mutex_lock(&_mutex_read);

    if(_waiting_read_counter<2){
        _waiting_read_counter++;
    }

    if (_waiting_read_counter == 2) {
        pthread_cond_broadcast(&read_cond);
        _waiting_read_counter=0;
    } else {
        pthread_cond_wait(&read_cond, &_mutex_read);
    }

    pthread_mutex_unlock(&_mutex_read);
}

void EcIface::sync_write(void) {
    
    pthread_mutex_lock(&_mutex_write);

    if(_waiting_write_counter<2){
        _waiting_write_counter++;
    }

    if (_waiting_write_counter == 2) {
        pthread_cond_broadcast(&write_cond);
        _waiting_write_counter=0;
    } else {
        pthread_cond_wait(&write_cond, &_mutex_write);
    }

    pthread_mutex_unlock(&_mutex_write);
}


void EcIface::get_motors_status(MotorStatusMap &motor_status_map)
{
    motor_status_map= _motor_status_map;
}

void EcIface::set_motors_references(const RefFlags motor_ref_flags,const MotorReferenceMap motors_references)
{
    int ret=check_maps(_motors_references,motors_references);
    if(ret==0){
        _motor_ref_flags = motor_ref_flags;
        _motors_references = motors_references;
    }
    else{
        if(ret==-1){
            DPRINTF("No motor detected!\n");
        }
        else if(ret==-2){
            DPRINTF("Got an empy motors references map\n");
        }
        else if(ret==-3){
            DPRINTF("Got an different motors references size\n");
        }
        else{
            DPRINTF("Esc id [%d] is not a motor\n",ret);
        }
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

void EcIface::set_valves_references(const RefFlags valve_ref_flags,const ValveReferenceMap valves_references)
{
    int ret=check_maps(_valves_references,valves_references);
    if(ret==0){
        _valve_ref_flags=valve_ref_flags;
        _valves_references=valves_references;
    }
    else{
        if(ret==-1){
            DPRINTF("No valve detected!\n");
        }
        else if(ret==-2){
            DPRINTF("Got an empy valves references map\n");
        }
        else if(ret==-3){
            DPRINTF("Got an different valves references size\n");
        }
        else{
            DPRINTF("Esc id [%d] is not a valve\n",ret);
        }
    }
}

void EcIface::get_pump_status(PumpStatusMap &pump_status_map)
{
    pump_status_map= _pump_status_map;
}

void EcIface::set_pumps_references(const RefFlags pump_ref_flags,const PumpReferenceMap pumps_references)
{
    int ret=check_maps(_pumps_references,pumps_references);
    if(ret==0){
        _pump_ref_flags=pump_ref_flags;
        _pumps_references=pumps_references;
    }
    else{
        if(ret==-1){
            DPRINTF("No pump detected!\n");
        }
        else if(ret==-2){
            DPRINTF("Got an empy pumps references map\n");
        }
        else if(ret==-3){
            DPRINTF("Got an different pumps references size\n");
        }
        else{
            DPRINTF("Esc id [%d] is not a pump\n",ret);
        }
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

template <typename T>
int32_t EcIface::check_maps(const std::map<int32_t,T>& map1,const std::map<int32_t,T>& map2)
{
    if(map1.size()==0)
        return -1;
    else if(map2.size()==0)
        return -2;
    else if(map1.size()!=map2.size())
        return -3;
    else{
        for ( const auto &[esc_id,tx_pdo] : map2) {
            if(map1.count(esc_id)==0){
                return esc_id; 
            }
        }
    }

    return 0;
}
