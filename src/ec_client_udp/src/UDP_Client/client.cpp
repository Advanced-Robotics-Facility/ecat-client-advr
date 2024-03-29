#include <cassert>
#include <tuple>

#include "udpSock.h"
#include "client.h"
#include "pck_msgs.h"

#include <magic_enum.hpp>

using boost::asio::ip::udp;
using namespace std::chrono_literals;
using namespace std::chrono;

#undef PROFILE_TIMING

/**
 * @brief Client::Client
 */
Client::Client(std::string host_address,uint32_t host_port) :
    UdpTask("Client", CLIENT_PORT),
    consoleLog{ spdlog::get("console") }
{
    consoleLog->info(" Client Started " + make_daytime_string());
    
    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    sender_endpoint.address(boost::asio::ip::address::from_string(host_address));
    sender_endpoint.port(host_port);

    // Register Message Handler
    registerHandler(UdpPackMsg::MSG_SRV_REP,    &Client::server_replies_handler);
    registerHandler(UdpPackMsg::MSG_REPL_REP,   &Client::repl_replies_handler);
    registerHandler(UdpPackMsg::MSG_SRV_STS,    &Client::server_status_handler);
    registerHandler(UdpPackMsg::MSG_MOTOR_STS,  &Client::motor_status_handler);
    registerHandler(UdpPackMsg::MSG_FT6_STS,    &Client::ft6_status_handler);
    registerHandler(UdpPackMsg::MSG_PWR_STS,    &Client::pwr_status_handler);

    
    _mutex_motor_status= std::make_shared<std::mutex>();
    
    _mutex_ft6_status= std::make_shared<std::mutex>();
    
    _mutex_pow_status= std::make_shared<std::mutex>();
    
    _cv_repl_reply= std::make_shared<std::condition_variable>();
    
    _cmd_req_reply=false;
    
    _reply_err_msg="";
    
    _wait_reply_time = 1000;  //1s
    
    _client_alive = true;
    _client_alive_time=steady_clock::now();
        
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500); //1.5s
    _actual_server_status= ServerStatus::IDLE;
    _client_status=ClientStatus::IDLE;
}



//******************************* EVENT HANDLERS *****************************************************//

void Client::server_replies_handler(char*buf, size_t size)
{   
    size_t offset {};
    auto reply = proto.getCliReqSrvRep(buf, size, offset);
    consoleLog->info( " SRV REP : {}", magic_enum::enum_name(reply));
    int64_t ts;
    int64_t usecs_since_epoch = getTsEpoch<std::chrono::microseconds>();
    SCA server_args;
    uint32_t hash;

    switch (reply) {
    
        case CliReqSrvRep::CONNECTED :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, server_args);
            std::tie(hash, std::ignore) = server_args;
            consoleLog->info(" <-- Connected ! {}", hash);
            _client_status=ClientStatus::CONNECTED;
            break;
    
        case CliReqSrvRep::PONG :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, ts);
            consoleLog->info( "PING PONG rtt {} us", usecs_since_epoch-ts);
            break;
    
        default:
            break;
    }
}


/**
 * @brief status_handler
 * @param buf
 * @param size
 */
void Client::server_status_handler(char *buf, size_t size)
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

/**
 * @brief repl_replies_handler
 * @param buf
 * @param size
 */
void Client::repl_replies_handler(char *buf, size_t size)
{
    _reply_err_msg="";
    
    int ret = proto.getReplReply(buf, size, _repl_req_rep, _reply_err_msg);
    consoleLog->info( "   REPL REP : {} {}", magic_enum::enum_name(_repl_req_rep), _reply_err_msg);

    switch ( _repl_req_rep ) {
    
        case ReplReqRep::SLAVES_INFO:
            {
                ret = proto.getReplReplySlaveInfo(buf,size,_slave_info,_reply_err_msg);
                for ( auto &[id, type, pos] : _slave_info ) {
                    consoleLog->info( "     id {} type {} pos {}", id, type, pos);
                }
            }
            break;

        case ReplReqRep::SDO_CMD:
            {
                uint32_t esc_id;
                _rr_sdo.clear();
                ret = proto.getReplReplySdoCmd(buf,size, esc_id, _rr_sdo,_reply_err_msg);
                for ( auto &[k,v] : _rr_sdo ) {
                    consoleLog->info( " <-- id {} : {} = {}", esc_id, k, v);
                }
            }
            break;

        default:
            break;
    }
    
    if(_cmd_req_reply)
    {
        _cv_repl_reply->notify_one();
        _cmd_req_reply=false;
    }
}


void Client::motor_status_handler(char *buf, size_t size)
{
    static uint32_t cnt;
    static MSS motors_status;
    
    _mutex_motor_status->lock();
    
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
                //consoleLog->info( " motor_status {}: {} {} {} {} ", cnt,esc_id, motor_pos, motor_vel,torque);
            } catch (std::out_of_range oor) {}
        }
    }
    
    _mutex_motor_status->unlock();

}

void Client::ft6_status_handler(char *buf, size_t size)
{
    static uint32_t cnt;
    static FTS fts_status;
    
    _mutex_ft6_status->lock();
    
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_FT6_STS, fts_status);

    cnt++;
    for ( const auto &[id, values] : fts_status) {
        _ft_status_map[id] = values;
    }

    if ( ! (cnt % 100) ) {
        for ( const auto &[esc_id, values] : _ft_status_map) {
            //consoleLog->info( " ft_status {}: {} {} ", cnt,esc_id,values[0] );
        }
    }
    
    _mutex_ft6_status->unlock();

}

void Client::pwr_status_handler(char *buf, size_t size)
{
    static uint32_t cnt;
    static PWS pow_status;
    
    _mutex_pow_status->lock();
    
    auto ret = proto.getEscStatus(buf,size,UdpPackMsg::MSG_PWR_STS, pow_status);

    cnt++;
    for ( const auto &[id, values] : pow_status) {
        _pow_status_map[id] = values;
    }

    if ( ! (cnt % 100) ) {
        for ( const auto &[esc_id, values] : _pow_status_map) {
            //consoleLog->info( " pwr_status {}: {} {} {} ", cnt,esc_id,values[0],values[1] );
        }
    }
    
    _mutex_pow_status->unlock();

}
//******************************* EVENT HANDLERS *****************************************************//


//******************************* COMMANDS *****************************************************//
void Client::connect()
{
    CBuff sendBuffer{};
    CCA client_args = std::make_tuple(CLIENT_PORT, get_period_ms());
    auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::CONNECT, client_args);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

void Client::disconnect()
{
    CBuff sendBuffer{};
    uint32_t payload = 0xCACA0;
    auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::DISCONNECT, payload);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

void Client::ping(bool test)
{
    CBuff sendBuffer{};
    int64_t microseconds_since_epoch = getTsEpoch<std::chrono::microseconds>();
    CliReqSrvRep cliReq = CliReqSrvRep::PING;
    if ( test ) {
        cliReq = CliReqSrvRep::PING_TEST;
    }  
    auto sizet = proto.packClientRequest(sendBuffer, cliReq, microseconds_since_epoch);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

void Client::quit_server()
{
    CBuff sendBuffer{};
    uint32_t payload = 0xC1A0C1A0;
    auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::QUIT, payload);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool Client::get_reply_from_server(ReplReqRep cmd_req)
{
    std::mutex _cv_m;
    std::unique_lock<std::mutex> lk(_cv_m);
    
    _cmd_req_reply=true;
    _cv_repl_reply->wait_for(lk,std::chrono::milliseconds(_wait_reply_time)); // timer for ACK and NACK from udp server
    
    if(_client_alive)
    {
        consoleLog->info(" Command requested ---> {} Command reply--->{} ", cmd_req, _repl_req_rep);
        if(cmd_req == _repl_req_rep)
        {
            if(_reply_err_msg == "OkI")
            {
                _reply_err_msg = "";
                return true;
            }
        }
    }

    return false;
}


/**
 * @brief slave_info_handler
 * @param buf
 * @param size
 */
void Client::get_slaves_info()
{
    CBuff sendBuffer{};
    auto sizet = proto.packReplRequest(sendBuffer, ReplReqRep::SLAVES_INFO);
    do_send(sendBuffer.data(), sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool Client::retrieve_slaves_info(SSI &slave_info)
{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    _slave_info.clear();
    while(slave_info.empty() && attemps_cnt < _max_cmd_attemps)
    {
        if(_client_alive)
        {
            get_slaves_info();

            ret_cmd_status = get_reply_from_server(ReplReqRep::SLAVES_INFO);
            if(ret_cmd_status)
            {
                slave_info = _slave_info;
                attemps_cnt = _max_cmd_attemps;
                _client_status=ClientStatus::MOTORS_MAPPED;
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            consoleLog->error("UDP client not alive, please stop the main process!");
            return false;
        }
    }
    
    return ret_cmd_status;
}

void Client::getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo)
{
    CBuffT<4096u> sendBuffer{};
    
    RDWR_SDO rdwr_sdo = std::make_tuple(esc_id,rd_sdo,wr_sdo);

    auto sizet = proto.packReplRequestSdoCmd (sendBuffer, rdwr_sdo);
    do_send(sendBuffer.data(), sendBuffer.size() );
    consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool Client::retrieve_rr_sdo(uint32_t esc_id,
                             const RD_SDO &rd_sdo, 
                             const WR_SDO &wr_sdo,
                             RR_SDO &rr_sdo)

{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    _rr_sdo.clear();
    while(rr_sdo.empty() && attemps_cnt < _max_cmd_attemps)
    {
        if(_client_alive)
        {
            getAndset_slaves_sdo(esc_id,rd_sdo,wr_sdo);
            
            ret_cmd_status = get_reply_from_server(ReplReqRep::SDO_CMD);
            if(ret_cmd_status)
            {
                rr_sdo = _rr_sdo;
                attemps_cnt = _max_cmd_attemps;
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    return ret_cmd_status;
}

bool Client::start_motors(const MST &motors_start)
{
    int attemps_cnt=0;
    bool ret_cmd_status=false;
    
    restore_wait_reply_time(); //restore default wait reply time.
    uint32_t extend_wait_reply_time = _wait_reply_time + 1000 * (motors_start.size() / 10 );
    set_wait_reply_time(extend_wait_reply_time);
    
    while(attemps_cnt < _max_cmd_attemps)
    {
        if(_client_alive)
        {
            CBuffT<4096u> sendBuffer{};
            auto sizet = proto.packReplRequestMotorsStart(sendBuffer, motors_start);
            do_send(sendBuffer.data(), sendBuffer.size() );
            consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
            ret_cmd_status = get_reply_from_server(ReplReqRep::START_MOTOR);
            
            if(ret_cmd_status)
            {
                attemps_cnt = _max_cmd_attemps;
                _client_status=ClientStatus::MOTORS_STARTED;
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    
    restore_wait_reply_time();
    
    return ret_cmd_status;
}


bool Client::stop_motors()
{
    int attemps_cnt=0;
    bool ret_cmd_status=false;
    while(attemps_cnt < _max_cmd_attemps)
    {
        if(_client_alive)
        {
            CBuff sendBuffer{};
            auto sizet = proto.packReplRequest(sendBuffer, ReplReqRep::STOP_MOTOR);
            do_send(sendBuffer.data(), sendBuffer.size() );
            consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
            
            ret_cmd_status = get_reply_from_server(ReplReqRep::STOP_MOTOR);
            if(ret_cmd_status)
            {
                attemps_cnt = _max_cmd_attemps;
                _client_status=ClientStatus::MOTORS_STOPPED;
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    
    return ret_cmd_status;
}

bool Client::pdo_aux_cmd(const PAC & pac)
{
    bool ret_cmd_status=false;
    if(_client_alive)
    {
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetPdoAuxCmd(sendBuffer, pac);
        do_send(sendBuffer.data(), sendBuffer.size() );
        consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        ret_cmd_status=true;
    }
    else
    {
        consoleLog->error("UDP client not alive, please stop the main process");
    }

    return ret_cmd_status;
}

bool Client::pdo_aux_cmd_sts(const PAC & pac)
{
    auto motor_status_map = get_motors_status();
    
    for( const auto &[esc_id,pdo_aux_cmd] : pac)
    {
        if(motor_status_map.count(esc_id) > 0)
        {
            auto motor_status = motor_status_map[esc_id];
            
            uint32_t cmd_aux_sts=std::get<11>(motor_status);
            

            uint32_t brake_sts = cmd_aux_sts & 3; //00 unknown
                                                    //01 release brake 
                                                    //10 enganged brake
                                                    //11 error
                                                    
            uint32_t led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
            
            
            switch(pdo_aux_cmd){
                
                case to_underlying(PdoAuxCmdType::BRAKE_RELEASE):{
                    if(brake_sts!=1){ 
                        consoleLog->error("esc_id: {}, brake status: {} ---> brake requested: {} ",esc_id, brake_sts, PdoAuxCmdType::BRAKE_RELEASE);
                        return false;
                    }
                }break;
                case to_underlying(PdoAuxCmdType::BRAKE_ENGAGE):{
                    if(brake_sts!=2){ 
                        consoleLog->error("esc_id: {}, brake status: {} ---> brake requested: {} ",esc_id, brake_sts, PdoAuxCmdType::BRAKE_ENGAGE);
                        return false;
                    }
                }break;
                case to_underlying(PdoAuxCmdType::LED_ON):{
                    if(led_sts!=1){ 
                        consoleLog->error("esc_id: {}, led status: {} ---> led requested: {} ",esc_id, led_sts, PdoAuxCmdType::LED_ON);
                        return false;
                    }
                }break;
                case to_underlying(PdoAuxCmdType::LED_OFF):{
                    if(led_sts!=0){ 
                        consoleLog->error("esc_id: {}, led status: {} ---> led requested: {} ",esc_id, led_sts, PdoAuxCmdType::LED_OFF);
                        return false;
                    }
                }break;
            }
        }
        else
        {
            consoleLog->error("esc_id: {}, doesn't exist, please restart the request", esc_id);
            return false; // return false if the esc id it's not present into the motor status map.
        }
    }
        
    return true;
}


void Client::feed_motors(const MSR & m_ref)
{
    if(_client_alive)
    {
        if(_client_status==ClientStatus::MOTORS_STARTED ||
           _client_status==ClientStatus::MOTORS_CTRL)
        {
            CBuffT<4096u> sendBuffer{};
            auto sizet = proto.packReplRequestSetMotorsRefs(sendBuffer, m_ref);
            do_send(sendBuffer.data(), sendBuffer.size() );
            consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        }
        else
        {
            consoleLog->error("Cannot send references to the motors since not controlled, stop sending them");
        }
    }
    else
    {
        consoleLog->error("UDP client not alive, please stop the main process");
    }
}


void Client::set_motors_gains(const MSG &motors_gains)
{
    if(_client_alive)
    {
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetMotorsGains(sendBuffer, motors_gains);
        do_send(sendBuffer.data(), sendBuffer.size() );
        consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
    }
    else
    {
        consoleLog->error("UDP client not alive, please stop the main process");
    }
}
//******************************* COMMANDS *****************************************************//


//******************************* Periodic Activity *****************************************************//
#ifdef PROFILE_TIMING
auto lastTime_ = std::chrono::high_resolution_clock::now();
#endif
/**
 * @brief Client::periodicActivity
 */
void Client::periodicActivity()
{
    auto sample_time = steady_clock::now();
    
    // Server alive checking //
    auto client_alive_elapsed_ms=duration_cast<milliseconds>(sample_time-_client_alive_time);
    
    if(_actual_server_status==ServerStatus::IDLE)
    {
        if(client_alive_elapsed_ms >= _server_alive_check_ms)
        {
             // Stop to receive motors, imu, ft, power board pdo information // 
            _client_alive =false;
            stop_client();
        }
    }
    else
    {
        _actual_server_status= ServerStatus::IDLE;
        _client_alive_time = steady_clock::now();
        
        // Receive motors, imu, ft, power board pdo information // 
    }
        
}
//******************************* Periodic Activity *****************************************************//

void Client::stop_client()
{
    spdlog::get("console")->info("That's all folks");
    
    if(_client_status!=ClientStatus::IDLE)
    {
        disconnect();
    }
    
    this->stop();
    
}

MotorStatusMap Client::get_motors_status()
{
    _mutex_motor_status->lock();
    
    auto ret_motor_status_map= _motor_status_map;
    
    _mutex_motor_status->unlock();
    
    return ret_motor_status_map;
}

FtStatusMap Client::get_ft6_status()
{
    _mutex_ft6_status->lock();
    
    auto ret_ft_status_map= _ft_status_map;
    
    _mutex_ft6_status->unlock();
    
    return ret_ft_status_map; 
}

PwrStatusMap Client::get_pow_status()
{
    _mutex_pow_status->lock();
    
    auto ret_pow_status_map= _pow_status_map;
    
    _mutex_pow_status->unlock();
    
    return ret_pow_status_map; 
}

void Client::set_wait_reply_time(uint32_t wait_reply_time)
{
    _wait_reply_time = wait_reply_time; // ms
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500);
}
void Client::restore_wait_reply_time()
{
    _wait_reply_time = 1000; //1s
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500); //1.5s
}

bool Client::is_client_alive()
{
    return _client_alive;
}

/**
 */
void Client::receive_error(std::error_code ec)
{
    consoleLog->error( " Receive Error {}", ec.message());
}



