#include "mechanism/boost/ec_boost_pdo.h"

void EcBoostPdo::esc_factory(SSI slave_descr)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        switch ( esc_type  )
        {
                case iit::ecat::CENT_AC :
                case iit::ecat::LO_PWR_DC_MC:
                case iit::ecat::SYNAPTICON_v5_0:
                case iit::ecat::SYNAPTICON_v5_1:{
                    _internal_motor_status_map[id]=_motor_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    _internal_motor_reference_map[id]=_motor_reference_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
                    if(ec_motors.count(esc_type)>0){
                        if(ec_motors[esc_type] == "ADVRF_Motor"){
                            _advrf_motor_map[id]=esc_type;
                        }
                    } 
                }break;
                case iit::ecat::FT6_MSP432:{
                    _internal_ft_status_map[id]=_ft_status_map[id]={0,0,0,0,0,0,0,0};
                }break;   
                case iit::ecat::IMU_ANY :{
                    _internal_imu_status_map[id]=_imu_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                    _internal_pow_status_map[id]=_pow_status_map[id]={0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::HYQ_KNEE:{
                    _internal_valve_status_map[id]=_valve_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0};
                    _internal_valve_reference_map[id]=_valve_reference_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::HYQ_HPU:{
                    _internal_pump_status_map[id]=_pump_status_map[id]={0,0,0,0,0,0,0,0,0,0,0};
                    _internal_pump_reference_map[id]=_pump_reference_map[id]={0,0,0,0,0,0,0,0,0,0};
                }break;
                
                default:
                    break;
        }               
    }
} 


//******************************* EVENT HANDLERS *****************************************************//


void EcBoostPdo::server_status_handler(char *buf, size_t size)
{
    _actual_server_status = proto.getServerStatus(buf,size);
    switch (_actual_server_status) {
        
        case ServerStatus::IDLE :
            break;
        case ServerStatus::CONNECTED :
            break;
        case ServerStatus::MOTOR_STARTED :{
            _client_status.status=ClientStatusEnum::DEVICES_STARTED;
        }break;
        case ServerStatus::MOTOR_STOPPED :{
            _client_status.status=ClientStatusEnum::DEVICES_STOPPED;
        }break;
        case ServerStatus::MOTOR_CTRL :{
            _client_status.status=ClientStatusEnum::DEVICES_CTRL;
        }break;
    
        default:
            break;
    }
}

void EcBoostPdo::motor_status_handler(char *buf, size_t size)
{
    static MSS motors_status;
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_MOTOR_STS, motors_status);

    // NOTE add extra PDO like pos_ref, vel_ref, tor_ref, curr_ref feedback
    for ( auto &[id,status_word,
                 link_pos,motor_pos,link_vel,motor_vel,
                 torque,current,motor_temp,board_temp,
                 fault,rtt,
                 pos_ref_fb,vel_ref_fb,tor_ref_fb,curr_ref_fb] : motors_status) {
        if(_internal_motor_status_map.count(id)>0){
            if(_advrf_motor_map.count(id)>0 && 
               _internal_motor_reference_map.count(id)>0){
                if(std::get<0>(_internal_motor_reference_map[id])==0xDD){
                    curr_ref_fb=tor_ref_fb;
                    tor_ref_fb=0.0;
                }
            }
            _internal_motor_status_map[id] = std::make_tuple(status_word,
                                                             link_pos,motor_pos,link_vel,motor_vel,
                                                             torque,current,motor_temp,board_temp,
                                                             fault,rtt,
                                                             pos_ref_fb,vel_ref_fb,tor_ref_fb,curr_ref_fb);
        }
        else{
            _consoleLog->error( "Id {} is not a motor",id);
        }
    }

    if(!_internal_motor_status_map.empty()){
        _motor_status_queue.push(_internal_motor_status_map);
    }
}

void EcBoostPdo::ft6_status_handler(char *buf, size_t size)
{
    static FTS fts_status;   
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_FT6_STS, fts_status);

    for ( const auto &[id, values] : fts_status) {
        if(_internal_ft_status_map.count(id)>0){
            _internal_ft_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                                          values[3],values[4],values[5],0,0);
        }
        else{
            _consoleLog->error( "Id {} is not a force/torque sensor",id);
        }
    }

    if(!_internal_ft_status_map.empty()){
        _ft_status_queue.push(_internal_ft_status_map);
    }
}

void EcBoostPdo::pwr_status_handler(char *buf, size_t size)
{    
    static PWS pow_status;
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_PWR_STS, pow_status);

    for ( const auto &[id, values] : pow_status) {
        if(_internal_pow_status_map.count(id)>0){
            _internal_pow_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                                  values[3],values[4],values[5],
                                                  0,0,0,0);
        }
        else{
            _consoleLog->error( "Id {} is not a power board",id);
        }
    }

    if(!_internal_pow_status_map.empty()){
        _pow_status_queue.push(_internal_pow_status_map);
    }
}
//******************************* EVENT HANDLERS *****************************************************//

