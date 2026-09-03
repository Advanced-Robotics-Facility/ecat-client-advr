#include "mechanism/boost/ec_boost_pdo.h"

void EcBoostPdo::esc_factory(SSI slave_descr)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        if(ec_motors().count(esc_type)>0){
            _motor_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            _motor_reference_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
        } else if(ec_valves().count(esc_type)>0){
            _valve_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0};
            _valve_reference_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
        } else if(ec_pumps().count(esc_type)>0){
            _pump_status_map[id]={0,0,0,0,0,0,0,0,0,0,0};
            _pump_reference_map[id]={0,0,0,0,0,0,0,0,0,0};
        } else if(ec_grippers().count(esc_type)>0){
            _gripper_status_map[id]={0,0,0,0,0,0,0};
            _gripper_reference_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
        } else{
            switch ( esc_type ){
                case iit::ecat::FT6MSP432_v24:{
                    _ft_status_map[id]={0,0,0,0,0,0,0,0};
                }break;   
                case iit::ecat::IMUVN :{
                    _imu_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::POWF28M36 :{
                    _pow_status_map[id]={0,0,0,0,0,0,0,0,0,0};
                }break;
                default:
                    break;
            } 
        }             
    }

    _internal_motor_status.resize(_motor_status_map.size());
    _internal_valve_status.resize(_valve_status_map.size());
    _internal_pump_status.resize(_pump_status_map.size());
    _internal_gripper_status.resize(_gripper_status_map.size());

    _internal_ft_status.resize(_ft_status_map.size());
    _internal_imu_status.resize(_imu_status_map.size());
    _internal_pow_status.resize(_pow_status_map.size());
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

        std::size_t index = 0;
        bool esc_found = false;
    
        for (const auto& [esc_id, motor_pdo] : _motor_status_map) {
            if (esc_id == id) {
                esc_found = true;
                break;
            }
    
            ++index;
        }

        if(esc_found){
            if(_motor_reference_map.count(id)>0){
                if(std::get<0>(_motor_reference_map[id])==0xDD){
                    curr_ref_fb=tor_ref_fb;
                    tor_ref_fb=0.0;
                }
            }
                
            _internal_motor_status[index] = std::make_tuple(status_word,
                                                            link_pos,motor_pos,link_vel,motor_vel,
                                                            torque,current,motor_temp,board_temp,
                                                            fault,rtt,
                                                            pos_ref_fb,vel_ref_fb,tor_ref_fb,curr_ref_fb);
        }
        else{
            _consoleLog->error( "Id {} is not a motor",id);
        }
    }

    if(!_internal_motor_status.empty()){
        _motor_status_queue.push(_internal_motor_status);
    }
}

void EcBoostPdo::ft6_status_handler(char *buf, size_t size)
{
    static FTS fts_status;   
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_FT6_STS, fts_status);

    for ( const auto &[id, values] : fts_status) {


        std::size_t index = 0;
        bool esc_found = false;
    
        for (const auto& [esc_id, ft_pdo] : _ft_status_map) {
            if (esc_id == id) {
                esc_found = true;
                break;
            }
    
            ++index;
        }

        if(esc_found){
            _internal_ft_status[index] = std::make_tuple(values[0],values[1],values[2],
                                                         values[3],values[4],values[5],0,0);
        }else{
            _consoleLog->error( "Id {} is not a force/torque sensor",id);
        }
    }

    if(!_internal_ft_status.empty()){
        _ft_status_queue.push(_internal_ft_status);
    }
}

void EcBoostPdo::pwr_status_handler(char *buf, size_t size)
{    
    static PWS pow_status;
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_PWR_STS, pow_status);

    for ( const auto &[id, values] : pow_status) {


        std::size_t index = 0;
        bool esc_found = false;

        for (const auto& [esc_id, pow_pdo] : _pow_status_map) {
            if (esc_id == id) {
                esc_found = true;
                break;
            }
    
            ++index;
        }

        if(esc_found){
            _internal_pow_status[index] = std::make_tuple(values[0],values[1],values[2],
                                                          values[3],values[4],values[5],
                                                          0,0,0,0);
        }else{
            _consoleLog->error( "Id {} is not a power board",id);
        }
    }

    if(!_internal_pow_status.empty()){
        _pow_status_queue.push(_internal_pow_status);
    }
}
//******************************* EVENT HANDLERS *****************************************************//

