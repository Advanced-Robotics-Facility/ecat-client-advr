#include <cassert>
#include <tuple>

#include "udpSock.h"
#include "protocols/udp/ec_udp.h"
#include "pck_msgs.h"

#include <magic_enum.hpp>

using boost::asio::ip::udp;
using namespace std::chrono_literals;
using namespace std::chrono;


/**
 * @brief EcUDP::EcUDP
 */
EcUDP::EcUDP(std::string host_address,uint32_t host_port) :
    UdpTask("EcUDP", CLIENT_PORT)
{

    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    
    sender_endpoint.address(boost::asio::ip::address::from_string(host_address));
    sender_endpoint.port(host_port);

    // Register Message Handler
    registerHandler(UdpPackMsg::MSG_SRV_REP,    &EcUDP::server_replies_handler);
    registerHandler(UdpPackMsg::MSG_REPL_REP,   &EcUDP::repl_replies_handler);
    registerHandler(UdpPackMsg::MSG_SRV_STS,    &EcUDP::server_status_handler);
    registerHandler(UdpPackMsg::MSG_MOTOR_STS,  &EcUDP::motor_status_handler);
    registerHandler(UdpPackMsg::MSG_FT6_STS,    &EcUDP::ft6_status_handler);
    registerHandler(UdpPackMsg::MSG_PWR_STS,    &EcUDP::pwr_status_handler);
    
    _cv_repl_reply= std::make_shared<std::condition_variable>();
    
    _cmd_req_reply=false;
    
    _reply_err_msg="";
    
    _wait_reply_time = 1000;  //1s
    
    _client_alive=true;
    _client_alive_time=steady_clock::now();
        
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500); //1.5s
    _actual_server_status= ServerStatus::IDLE;
}

EcUDP::~EcUDP()
{
    stop_client();
    
    if(_ec_udp_thread != nullptr)
    {
        if ( _ec_udp_thread->joinable() ) 
        {
            _consoleLog->info("EtherCAT Client thread stopped");
            _ec_udp_thread->join();
        }
    }
    
    _consoleLog->info("That's all folks");
    _consoleLog.reset();
}


//******************************* EVENT HANDLERS *****************************************************//

void EcUDP::server_replies_handler(char*buf, size_t size)
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


/**
 * @brief status_handler
 * @param buf
 * @param size
 */
void EcUDP::server_status_handler(char *buf, size_t size)
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
void EcUDP::repl_replies_handler(char *buf, size_t size)
{
    _reply_err_msg="";
    
    int ret = proto.getReplReply(buf, size, _repl_req_rep, _reply_err_msg);
    _consoleLog->info( "   REPL REP : {} {}", magic_enum::enum_name(_repl_req_rep), _reply_err_msg);

    switch ( _repl_req_rep ) {
    
        case ReplReqRep::SLAVES_INFO:
            {
                ret = proto.getReplReplySlaveInfo(buf,size,_slave_info,_reply_err_msg);
                for ( auto &[id, type, pos] : _slave_info ) {
                    _consoleLog->info( "     id {} type {} pos {}", id, type, pos);
                }
            }
            break;

        case ReplReqRep::SDO_CMD:
            {
                uint32_t esc_id;
                _rr_sdo.clear();
                ret = proto.getReplReplySdoCmd(buf,size, esc_id, _rr_sdo,_reply_err_msg);
                for ( auto &[k,v] : _rr_sdo ) {
                    _consoleLog->info( " <-- id {} : {} = {}", esc_id, k, v);
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


void EcUDP::motor_status_handler(char *buf, size_t size)
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

void EcUDP::ft6_status_handler(char *buf, size_t size)
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

void EcUDP::pwr_status_handler(char *buf, size_t size)
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


//******************************* COMMANDS *****************************************************//
void EcUDP::connect()
{
    if(_client_status==ClientStatus::IDLE)
    {
        CBuff sendBuffer{};
        CCA client_args = std::make_tuple(CLIENT_PORT, get_period_ms());
        auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::CONNECT, client_args);
        do_send(sendBuffer.data(),  sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
    }
    else
    {
        _consoleLog->info("Client already connected, cannot perform the connect command");
    }

}

void EcUDP::disconnect()
{
    if(_client_status!=ClientStatus::IDLE)
    {
        CBuff sendBuffer{};
        uint32_t payload = 0xCACA0;
        auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::DISCONNECT, payload);
        do_send(sendBuffer.data(),  sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        _client_status=ClientStatus::IDLE;
    }
}

void EcUDP::ping(bool test)
{
    CBuff sendBuffer{};
    int64_t microseconds_since_epoch = getTsEpoch<std::chrono::microseconds>();
    CliReqSrvRep cliReq = CliReqSrvRep::PING;
    if ( test ) {
        cliReq = CliReqSrvRep::PING_TEST;
    }  
    auto sizet = proto.packClientRequest(sendBuffer, cliReq, microseconds_since_epoch);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

void EcUDP::quit_server()
{
    CBuff sendBuffer{};
    uint32_t payload = 0xC1A0C1A0;
    auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::QUIT, payload);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcUDP::get_reply_from_server(ReplReqRep cmd_req)
{
    std::mutex _cv_m;
    std::unique_lock<std::mutex> lk(_cv_m);
    
    _cmd_req_reply=true;
    _cv_repl_reply->wait_for(lk,std::chrono::milliseconds(_wait_reply_time)); // timer for ACK and NACK from udp server
    
    if(_client_alive)
    {
        _consoleLog->info(" Command requested ---> {} Command reply--->{} ", cmd_req, _repl_req_rep);
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
void EcUDP::get_slaves_info()
{
    CBuff sendBuffer{};
    auto sizet = proto.packReplRequest(sendBuffer, ReplReqRep::SLAVES_INFO);
    do_send(sendBuffer.data(), sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcUDP::retrieve_slaves_info(SSI &slave_info)
{

    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    if(!_slave_info.empty())
    {
        slave_info=_slave_info;
        return true;
    }

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
                if(_client_status!=ClientStatus::MOTORS_STARTED ||
                   _client_status!=ClientStatus::MOTORS_CTRL)
                {
                    _client_status=ClientStatus::MOTORS_MAPPED;
                }
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            _consoleLog->error("UDP client not alive, please stop the main process!");
            return false;
        }
    }
    
    return ret_cmd_status;
}

void EcUDP::getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo)
{
    CBuffT<4096u> sendBuffer{};
    
    RDWR_SDO rdwr_sdo = std::make_tuple(esc_id,rd_sdo,wr_sdo);

    auto sizet = proto.packReplRequestSdoCmd (sendBuffer, rdwr_sdo);
    do_send(sendBuffer.data(), sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcUDP::retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo)
{
    return false;
}

bool EcUDP::retrieve_rr_sdo(uint32_t esc_id,
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
            _consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    return ret_cmd_status;
}

bool EcUDP::set_wr_sdo(uint32_t esc_id,
                        const RD_SDO &rd_sdo,
                        const WR_SDO &wr_sdo)

{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;

    while(attemps_cnt < _max_cmd_attemps)
    {
        if(_client_alive)
        {
            getAndset_slaves_sdo(esc_id,rd_sdo,wr_sdo);

            ret_cmd_status = get_reply_from_server(ReplReqRep::SDO_CMD);
            if(ret_cmd_status)
            {
                attemps_cnt = _max_cmd_attemps;
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            _consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    return ret_cmd_status;
}

bool EcUDP::start_motors(const MST &motors_start)
{
    bool ret_cmd_status=false;

    if(_client_status==ClientStatus::MOTORS_STARTED)
    {
        _consoleLog->error("Motors already started, stop the motors before performing start motors command");
        return ret_cmd_status;
    }
    else if(_client_status==ClientStatus::MOTORS_CTRL)
    {
        _consoleLog->error("Motors are controlled, stop the motors before performing start motors command");
        return ret_cmd_status;
    }
    else
    {
        int attemps_cnt=0;
        restore_wait_reply_time(); //restore default wait reply time.
        uint32_t extend_wait_reply_time = _wait_reply_time + 1000 * (motors_start.size() / 10 );
        set_wait_reply_time(extend_wait_reply_time);

        while(attemps_cnt < _max_cmd_attemps)
        {
            if(_client_alive)
            {
                pthread_mutex_lock(&_mutex_motor_reference);

                // clear motors references
                _motor_ref_flags=RefFlags::FLAG_NONE;
                //_motors_references.clear();

                 pthread_mutex_unlock(&_mutex_motor_reference);

                CBuffT<4096u> sendBuffer{};
                auto sizet = proto.packReplRequestMotorsStart(sendBuffer, motors_start);
                do_send(sendBuffer.data(), sendBuffer.size() );
                _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
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
                _consoleLog->error("UDP client not alive, please stop the main process");
                return false;
            }
        }

        restore_wait_reply_time();
    }
    
    return ret_cmd_status;
}


bool EcUDP::stop_motors()
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
            _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
            
            ret_cmd_status = get_reply_from_server(ReplReqRep::STOP_MOTOR);
            if(ret_cmd_status)
            {
                attemps_cnt = _max_cmd_attemps;

                pthread_mutex_lock(&_mutex_motor_reference);
                _client_status=ClientStatus::MOTORS_STOPPED;

                // clear motors references
                _motor_ref_flags=RefFlags::FLAG_NONE;
                //_motors_references.clear();

                 pthread_mutex_unlock(&_mutex_motor_reference);
            }
            else
            {
                attemps_cnt++;
            }
        }
        else
        {
            _consoleLog->error("UDP client not alive, please stop the main process");
            return false;
        }
    }
    
    return ret_cmd_status;
}

bool EcUDP::pdo_aux_cmd(const PAC & pac)
{
    bool ret_cmd_status=false;
    if(_client_alive)
    {
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetPdoAuxCmd(sendBuffer, pac);
        do_send(sendBuffer.data(), sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        ret_cmd_status=true;
    }
    else
    {
        _consoleLog->error("UDP client not alive, please stop the main process");
    }

    return ret_cmd_status;
}


void EcUDP::feed_motors(const MSR & m_ref)
{
    if(_client_alive)
    {
        if(_client_status==ClientStatus::MOTORS_STARTED ||
           _client_status==ClientStatus::MOTORS_CTRL)
        {
            CBuffT<4096u> sendBuffer{};
            auto sizet = proto.packReplRequestSetMotorsRefs(sendBuffer, m_ref);
            do_send(sendBuffer.data(), sendBuffer.size() );
            _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
            _ec_logger->log_motors_ref(_motors_references);
        }
        else
        {
            _consoleLog->error("Cannot send references to the motors since not controlled, stop sending them");
        }
    }
    else
    {
        _consoleLog->error("UDP client not alive, please stop the main process");
    }
}

void EcUDP::set_motors_gains(const MSG &motors_gains)
{
    if(_client_alive)
    {
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetMotorsGains(sendBuffer, motors_gains);
        do_send(sendBuffer.data(), sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
    }
    else
    {
        _consoleLog->error("UDP client not alive, please stop the main process");
    }
}
//******************************* COMMANDS *****************************************************//


void EcUDP::set_loop_time(uint32_t period_ms)
{
    auto period_ms_time=milliseconds(period_ms);
    
    set_period(period_ms_time);
}

void EcUDP::start_client(uint32_t period_ms,bool logging)
{

    _consoleLog->info(" EtherCAT Client UDP Started " + make_daytime_string());
    
    set_loop_time(period_ms);
        
    connect();
    
    if(logging)
    {
        _ec_logger->init_mat_logger(_slave_info);
        start_logging();
    }
        
    if(_ec_udp_thread == nullptr)
    {
        _ec_udp_thread = std::make_shared<std::thread>(std::thread{[&]{run();}});
    }
    else
    {
        // leave the periodicActivity alive with a period changed
    }
}

void EcUDP::stop_client()
{
    if(_client_alive)
    {
        stop_logging();
        
        stop();
        
        disconnect();
        
        _client_alive =false;
    }
}



//******************************* Periodic Activity *****************************************************//
#ifdef PROFILE_TIMING
auto lastTime_ = std::chrono::high_resolution_clock::now();
#endif
/**
 * @brief Client::periodicActivity
 */
void EcUDP::periodicActivity()
{
    auto sample_time = steady_clock::now();
    
    // Server alive checking //
    auto client_alive_elapsed_ms=duration_cast<milliseconds>(sample_time-_client_alive_time);
    
    if(_actual_server_status==ServerStatus::IDLE)
    {
        if(client_alive_elapsed_ms >= _server_alive_check_ms)
        {
             // Stop to receive motors, imu, ft, power board pdo information // 
            // reset motors references
            _motor_ref_flags = RefFlags::FLAG_NONE;
            //_motors_references.clear();
            
            // stop client
            stop_client();
        }
    }
    else
    {
        _actual_server_status= ServerStatus::IDLE;
        _client_alive_time = steady_clock::now();
        
        // Receive motors, imu, ft, power board pdo information // 

        // Send motors references
        pthread_mutex_lock(&_mutex_motor_reference);

        if(_motor_ref_flags!=RefFlags::FLAG_NONE &&
           !_motors_references.empty())
        {
            std::vector<MR> mot_ref_v;
            for ( const auto &[bId,motor_tx] : _motors_references ) {
                auto ctrl_type=std::get<0>(motor_tx);
                if(ctrl_type!=0x00){
                    mot_ref_v.push_back(std::tuple_cat(std::make_tuple(bId),motor_tx));
                }
            }
            if(_motor_ref_flags==RefFlags::FLAG_MULTI_REF )
            {
                feed_motors(std::make_tuple(1, mot_ref_v));
            }
            else
            {
                feed_motors(std::make_tuple(2, mot_ref_v));
            }
        }

        pthread_mutex_unlock(&_mutex_motor_reference);
    }
        
}
//******************************* Periodic Activity *****************************************************//

void EcUDP::set_wait_reply_time(uint32_t wait_reply_time)
{
    _wait_reply_time = wait_reply_time; // ms
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500);
}
void EcUDP::restore_wait_reply_time()
{
    _wait_reply_time = 1000; //1s
    _server_alive_check_ms=milliseconds(_wait_reply_time)+milliseconds(500); //1.5s
}

void EcUDP::receive_error(std::error_code ec)
{
    _consoleLog->error( " Receive Error {}", ec.message());
}



