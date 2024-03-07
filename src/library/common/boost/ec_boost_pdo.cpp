#include <cassert>
#include <tuple>

#include "udpSock.h"
#include "common/boost/ec_boost_pdo.h"
#include <magic_enum.hpp>

/**
 * @brief EcBoostPdo::EcBoostPdo
 */
EcBoostPdo::EcBoostPdo(std::string task_name,std::string host_address,uint32_t host_port) :
                       UdpTask(task_name, host_port+2)
{

    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    
    sender_endpoint.address(boost::asio::ip::address::from_string(host_address));
    sender_endpoint.port(host_port);

    // Register Message Handler
    registerHandler(UdpPackMsg::MSG_SRV_REP,    &EcBoostPdo::server_replies_handler);
    registerHandler(UdpPackMsg::MSG_SRV_STS,    &EcBoostPdo::server_status_handler);
    registerHandler(UdpPackMsg::MSG_MOTOR_STS,  &EcBoostPdo::motor_status_handler);
    registerHandler(UdpPackMsg::MSG_FT6_STS,    &EcBoostPdo::ft6_status_handler);
    registerHandler(UdpPackMsg::MSG_PWR_STS,    &EcBoostPdo::pwr_status_handler);
}

EcBoostPdo::~EcBoostPdo()
{
    
}

//******************************* EVENT HANDLERS *****************************************************//

void EcBoostPdo::server_replies_handler(char*buf, size_t size)
{   
    size_t offset {};
    auto reply = proto.getCliReqSrvRep(buf, size, offset);
    _consoleLog->info( " SRV REP : {}", magic_enum::enum_name(reply));
    int64_t ts;
    int64_t usecs_since_epoch = getTsEpoch<std::chrono::microseconds>();
    SCA server_args;
    uint32_t hash;

    switch (reply) {
    
        case CliReqSrvRep::CONNECTED :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, server_args);
            std::tie(hash, std::ignore) = server_args;
            _consoleLog->info(" <-- Connected ! {}", hash);
            _client_status=ClientStatus::CONNECTED;
            break;
    
        case CliReqSrvRep::PONG :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, ts);
            _consoleLog->info( "PING PONG rtt {} us", usecs_since_epoch-ts);
            break;
    
        default:
            break;
    }
}

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
    static uint32_t cnt;
    static MSS motors_status;
    
    pthread_mutex_lock(&_mutex_motor_status);
    
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_MOTOR_STS, motors_status);

    cnt++;
    for ( const auto &[id,link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts] : motors_status) {
        _motor_status_map[id] = std::make_tuple(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts);
    }

    if ( ! (cnt % 100) ) {
        for ( const auto &[esc_id, values] : _motor_status_map) {
            try {
                float link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,aux;
                uint32_t fault,rtt,op_idx_ack,cmd_aux_sts;
                std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = values;
                //_consoleLog->info( " motor_status {}: {} {} {} {} ", cnt,esc_id, motor_pos, motor_vel,torque);
            } catch (std::out_of_range oor) {}
        }
    }
    
    _ec_logger->log_motors_sts(_motor_status_map);
    
    pthread_mutex_unlock(&_mutex_motor_status);

}

void EcBoostPdo::ft6_status_handler(char *buf, size_t size)
{
    static uint32_t cnt;
    static FTS fts_status;
    
   pthread_mutex_lock(&_mutex_ft_status);
    
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_FT6_STS, fts_status);

    cnt++;
    for ( const auto &[id, values] : fts_status) {
        _ft_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                             values[3],values[4],values[5],0,0);
    }
                 

    if ( ! (cnt % 100) ) {
        for ( const auto &[esc_id, values] : _ft_status_map) {
            //_consoleLog->info( " ft_status {}: {} {} ", cnt,esc_id,values[0] );
        }
    }
    
    _ec_logger->log_ft_sts(_ft_status_map);
    
    pthread_mutex_unlock(&_mutex_ft_status);

}

void EcBoostPdo::pwr_status_handler(char *buf, size_t size)
{
    static uint32_t cnt;
    static PWS pow_status;
    
    pthread_mutex_lock(&_mutex_pow_status);
    
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_PWR_STS, pow_status);

    cnt++;
    for ( const auto &[id, values] : pow_status) {
        _pow_status_map[id] = std::make_tuple(values[0],values[1],values[2],
                                              values[3],values[4],values[5],
                                              0,0,0,0);
    }

    if ( ! (cnt % 100) ) {
        for ( const auto &[esc_id, values] : _pow_status_map) {
            //_consoleLog->info( " pwr_status {}: {} {} {} ", cnt,esc_id,values[0],values[1] );
        }
    }
    
    _ec_logger->log_pow_sts(_pow_status_map);
    
    pthread_mutex_unlock(&_mutex_pow_status);

}
//******************************* EVENT HANDLERS *****************************************************//

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
                    _pump_status_map[id]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    _pumps_references[id]={0,0,0,0,0,0,0,0,0};
                }break;
                
                default:
                    break;
        }               
    }
} 


