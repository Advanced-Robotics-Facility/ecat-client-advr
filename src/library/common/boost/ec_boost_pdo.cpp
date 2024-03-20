#include "common/boost/ec_boost_pdo.h"

void EcBoostPdo::esc_factory(SSI slave_descr)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        switch ( esc_type  )
        {
                case iit::ecat::CENT_AC :
                case iit::ecat::LO_PWR_DC_MC:
                case iit::ecat::CIRCULO9:
                case iit::ecat::AMC_FLEXPRO:{
                    _motor_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0};
                    _motors_references[id]={0,0,0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::FT6_MSP432:{
                    _ft_status_map[id]={0,0,0,0,0,0,0,0};
                }break;   
                case iit::ecat::IMU_ANY :{
                    _imu_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                    _pow_status_map[id]={0,0,0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::HYQ_KNEE:{
                    _valve_status_map[id]={0,0,0,0,0,0,0,0,0};
                    _valves_references[id]={0,0,0,0,0,0,0,0};
                }break;
                case iit::ecat::HYQ_HPU:{
                    _pump_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    _pumps_references[id]={0,0,0,0,0,0,0,0,0};
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
            _client_status=ClientStatus::MOTORS_STARTED;
        }break;
        case ServerStatus::MOTOR_STOPPED :{
            _client_status=ClientStatus::MOTORS_STOPPED;
        }break;
        case ServerStatus::MOTOR_CTRL :{
            _client_status=ClientStatus::MOTORS_CTRL;
        }break;
    
        default:
            break;
    }
}

void EcBoostPdo::motor_status_handler(char *buf, size_t size)
{
    pthread_mutex_lock(&_mutex_motor_status);
    
    static MSS motors_status;
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_MOTOR_STS, motors_status);

    for ( const auto &[id,link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts] : motors_status) {
        if(_motor_status_map.count(id)>0){
            _motor_status_map[id] = std::make_tuple(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts);
        }
        else{
            _consoleLog->error( "Id {} is not a motor",id);
        }
    }
    _ec_logger->log_motors_sts(_motor_status_map);
    
    pthread_mutex_unlock(&_mutex_motor_status);

}

void EcBoostPdo::ft6_status_handler(char *buf, size_t size)
{
    pthread_mutex_lock(&_mutex_ft_status);
    
    static FTS fts_status;   
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_FT6_STS, fts_status);

    for ( const auto &[id, values] : fts_status) {
        if(_ft_status_map.count(id)>0){
            _ft_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                                 values[3],values[4],values[5],0,0);
        }
        else{
            _consoleLog->error( "Id {} is not a force/torque sensor",id);
        }
    }
    _ec_logger->log_ft_sts(_ft_status_map);
    
    pthread_mutex_unlock(&_mutex_ft_status);

}

void EcBoostPdo::pwr_status_handler(char *buf, size_t size)
{    
    pthread_mutex_lock(&_mutex_pow_status);
    
    static PWS pow_status;
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_PWR_STS, pow_status);

    for ( const auto &[id, values] : pow_status) {
        if(_pow_status_map.count(id)>0){
            _pow_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                                  values[3],values[4],values[5],
                                                  0,0,0,0);
        }
        else{
            _consoleLog->error( "Id {} is not a power board",id);
        }
    }
    _ec_logger->log_pow_sts(_pow_status_map);
    
    pthread_mutex_unlock(&_mutex_pow_status);

}
//******************************* EVENT HANDLERS *****************************************************//

