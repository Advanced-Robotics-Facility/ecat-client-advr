#include "mechanism/boost/ec_boost_cmd.h"

/**
 * @brief EcBoostCmd::EcBoostCmd
 */
EcBoostCmd::EcBoostCmd()
{
    _cv_repl_reply= std::make_shared<std::condition_variable>();
    
    _cmd_req_reply=false;
    
    _reply_err_msg="";
    
    _wait_reply_time = 1000;  //1s
    
    _server_alive_check_ms=std::chrono::milliseconds(_wait_reply_time)+std::chrono::milliseconds(500); //1.5s
}

EcBoostCmd::~EcBoostCmd()
{

}

//******************************* EVENT HANDLERS *****************************************************//

void EcBoostCmd::server_replies_handler(char*buf, size_t size)
{   
    size_t offset {};
    auto reply = proto.getCliReqSrvRep(buf, size, offset);
    _consoleLog->info( " SRV REP : {}", magic_enum::enum_name(reply));
    int64_t ts=0;
    int64_t usecs_since_epoch = getTsEpoch<std::chrono::microseconds>();
    SCA server_args;
    uint32_t hash;

    switch (reply) {
    
        case CliReqSrvRep::CONNECTED :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, server_args);
            std::tie(hash, std::ignore) = server_args;
            _consoleLog->info(" <-- Connected ! {}", hash);
            _client_status.status=ClientStatusEnum::CONNECTED;
            break;
    
        case CliReqSrvRep::PONG :
            this->proto.getCliReqSrvRepPayload(buf, size, offset, ts);
            _consoleLog->info( "PING PONG rtt {} us", usecs_since_epoch-ts);
            break;
    
        default:
            break;
    }
}

void EcBoostCmd::repl_replies_handler(char *buf, size_t size)
{
    _reply_err_msg="";
    
    int ret = proto.getReplReply(buf, size, _repl_req_rep, _reply_err_msg);
    _consoleLog->info( "   REPL REP : {} {}", magic_enum::enum_name(_repl_req_rep), _reply_err_msg);

    switch ( _repl_req_rep ) {
    
        case ReplReqRep::SLAVES_INFO:{
                ret = proto.getReplReplySlaveInfo(buf,size,_slave_info,_reply_err_msg);
                for ( auto &[id, type, pos] : _slave_info ) {
                    _consoleLog->info( "     id {} type {} pos {}", id, type, pos);
                }
        }break;
        case ReplReqRep::SDO_CMD:{
                uint32_t esc_id;
                _rr_sdo.clear();
                ret = proto.getReplReplySdoCmd(buf,size, esc_id, _rr_sdo,_reply_err_msg);
                for ( auto &[k,v] : _rr_sdo ) {
                    _consoleLog->info( " <-- id {} : {} = {}", esc_id, k, v);
                }
        }break;

        default:
            break;
    }
    
    if(_cmd_req_reply){
        _cv_repl_reply->notify_one();
        _cmd_req_reply=false;
    }
}

//******************************* COMMANDS *****************************************************//
void EcBoostCmd::connect()
{
    if(_client_status.status==ClientStatusEnum::IDLE){
        CBuff sendBuffer{};
        _consoleLog->info(" Client port:{} and period_ms: {} \n", _client_port, get_period_ms());
        CCA client_args = std::make_tuple(_client_port, get_period_ms());
        auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::CONNECT, client_args);
        do_send(sendBuffer.data(),  sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
    }
    else{
        _consoleLog->info("Client already connected, cannot perform the connect command");
    }

}

void EcBoostCmd::disconnect()
{
    if(_client_status.status!=ClientStatusEnum::IDLE){
        CBuff sendBuffer{};
        uint32_t payload = 0xCACA0;
        auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::DISCONNECT, payload);
        do_send(sendBuffer.data(),  sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        _client_status.status=ClientStatusEnum::IDLE;
    }
    else{
        _consoleLog->info("Client already disconnected, cannot perform the connect command");
    }
}

void EcBoostCmd::ping(bool test)
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

void EcBoostCmd::quit_server()
{
    CBuff sendBuffer{};
    uint32_t payload = 0xC1A0C1A0;
    auto sizet = proto.packClientRequest(sendBuffer, CliReqSrvRep::QUIT, payload);
    do_send(sendBuffer.data(),  sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}


bool EcBoostCmd::get_reply_from_server(ReplReqRep cmd_req)
{
    std::mutex _cv_m;
    std::unique_lock<std::mutex> lk(_cv_m);
    
    _cmd_req_reply=true;
    _cv_repl_reply->wait_for(lk,std::chrono::milliseconds(_wait_reply_time)); // timer for ACK and NACK from udp server
    
    if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
        _consoleLog->info(" Command requested ---> {} Command reply--->{} ", cmd_req, _repl_req_rep);
        if(cmd_req == _repl_req_rep){
            if(_reply_err_msg == "OkI"){
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
void EcBoostCmd::get_slaves_info()
{
    CBuff sendBuffer{};
    auto sizet = proto.packReplRequest(sendBuffer, ReplReqRep::SLAVES_INFO);
    do_send(sendBuffer.data(), sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcBoostCmd::retrieve_slaves_info(SSI &slave_info)
{

    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    if(!_slave_info.empty()){
        slave_info=_slave_info;
        return true;
    }

    _slave_info.clear();
    while(slave_info.empty() && attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            
            get_slaves_info();
            ret_cmd_status = get_reply_from_server(ReplReqRep::SLAVES_INFO);
            
            if(ret_cmd_status){
                slave_info = _slave_info;
                attemps_cnt = _max_cmd_attemps;
                if(_client_status.status!=ClientStatusEnum::DEVICES_STARTED ||
                   _client_status.status!=ClientStatusEnum::DEVICES_CTRL){
                    _client_status.status=ClientStatusEnum::DEVICES_MAPPED;
                }
            }
            else{
                attemps_cnt++;
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }
    
    if(_slave_info.empty()){
        if(!_fake_slave_info.empty()){
            _slave_info=_fake_slave_info;
            slave_info = _slave_info;
            ret_cmd_status=true;
        }     
    }

    return ret_cmd_status;
}

void EcBoostCmd::getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo)
{
    CBuffT<4096u> sendBuffer{};
    
    RDWR_SDO rdwr_sdo = std::make_tuple(esc_id,rd_sdo,wr_sdo);

    auto sizet = proto.packReplRequestSdoCmd (sendBuffer, rdwr_sdo);
    do_send(sendBuffer.data(), sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcBoostCmd::retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo)
{
    return false;
}

bool EcBoostCmd::retrieve_rr_sdo(uint32_t esc_id,
                             const RD_SDO &rd_sdo, 
                             const WR_SDO &wr_sdo,
                             RR_SDO &rr_sdo)

{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    _rr_sdo.clear();
    while(rr_sdo.empty() && attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            getAndset_slaves_sdo(esc_id,rd_sdo,wr_sdo);
            
            ret_cmd_status = get_reply_from_server(ReplReqRep::SDO_CMD);
            if(ret_cmd_status){
                rr_sdo = _rr_sdo;
                attemps_cnt = _max_cmd_attemps;
            }
            else{
                attemps_cnt++;
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }
    return ret_cmd_status;
}

bool EcBoostCmd::set_wr_sdo(uint32_t esc_id,
                        const RD_SDO &rd_sdo,
                        const WR_SDO &wr_sdo)

{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;

    while(attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            getAndset_slaves_sdo(esc_id,rd_sdo,wr_sdo);

            ret_cmd_status = get_reply_from_server(ReplReqRep::SDO_CMD);
            if(ret_cmd_status){
                attemps_cnt = _max_cmd_attemps;
            }
            else{
                attemps_cnt++;
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }
    return ret_cmd_status;
}

bool EcBoostCmd::start_devices(const DST &devices_start)
{
    bool ret_cmd_status=false;
    if(_client_status.status==ClientStatusEnum::DEVICES_STARTED){
        _consoleLog->error("Devices already started, stop the devices before performing start devices command");
        return ret_cmd_status;
    }
    else if(_client_status.status==ClientStatusEnum::DEVICES_CTRL){
        _consoleLog->error("Devices are controlled, stop the devices before performing start devices command");
        return ret_cmd_status;
    }
    else{
        int attemps_cnt=0;
        restore_wait_reply_time(); //restore default wait reply time.
        uint32_t extend_wait_reply_time = _wait_reply_time + 1000 * (devices_start.size() / 10 );
        set_wait_reply_time(extend_wait_reply_time);

        while(attemps_cnt < _max_cmd_attemps){
            if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
                CBuffT<4096u> sendBuffer{};
                auto sizet = proto.packReplRequestMotorsStart(sendBuffer, devices_start);
                do_send(sendBuffer.data(), sendBuffer.size() );
                _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
                ret_cmd_status = get_reply_from_server(ReplReqRep::START_MOTOR);

                if(ret_cmd_status){
                    attemps_cnt = _max_cmd_attemps;
                    _client_status.status=ClientStatusEnum::DEVICES_STARTED;
                }
                else{
                    attemps_cnt++;
                }
            }
            else{
                _consoleLog->error("Client in not alive state, please stop the main process!");
                return false;
            }
        }

        restore_wait_reply_time();
    }
    
    return ret_cmd_status;
}


bool EcBoostCmd::stop_devices()
{
    int attemps_cnt=0;
    bool ret_cmd_status=false;
    while(attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            CBuff sendBuffer{};
            auto sizet = proto.packReplRequest(sendBuffer, ReplReqRep::STOP_MOTOR);
            do_send(sendBuffer.data(), sendBuffer.size() );
            _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
            
            ret_cmd_status = get_reply_from_server(ReplReqRep::STOP_MOTOR);
            if(ret_cmd_status){
                attemps_cnt = _max_cmd_attemps;
                _client_status.status=ClientStatusEnum::DEVICES_STOPPED;
            }
            else{
                attemps_cnt++;
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }
    
    return ret_cmd_status;
}

bool EcBoostCmd::pdo_aux_cmd(const PAC & pac)
{
    bool ret_cmd_status=false;
    if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetPdoAuxCmd(sendBuffer, pac);
        do_send(sendBuffer.data(), sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
        ret_cmd_status=true;
    }
    else{
        _consoleLog->error("Client in not alive state, please stop the main process!");
    }

    return ret_cmd_status;
}


void EcBoostCmd::set_motors_gains(const MSG &motors_gains)
{
    if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
        CBuffT<4096u> sendBuffer{};
        auto sizet = proto.packReplRequestSetMotorsGains(sendBuffer, motors_gains);
        do_send(sendBuffer.data(), sendBuffer.size() );
        _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
    }
    else{
        _consoleLog->error("Client in not alive state, please stop the main process!");
    }
}
//******************************* COMMANDS *****************************************************//


void EcBoostCmd::set_wait_reply_time(uint32_t wait_reply_time)
{
    _wait_reply_time = wait_reply_time; // ms
    _server_alive_check_ms=std::chrono::milliseconds(_wait_reply_time)+std::chrono::milliseconds(500);
}
void EcBoostCmd::restore_wait_reply_time()
{
    _wait_reply_time = 1000; //1s
    _server_alive_check_ms=std::chrono::milliseconds(_wait_reply_time)+std::chrono::milliseconds(500); //1.5s
}


void EcBoostCmd::send_pdo()
{
    feed_motors();
}

void EcBoostCmd::feed_motors()
{
    if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
        if(_client_status.status==ClientStatusEnum::DEVICES_STARTED||
           _client_status.status==ClientStatusEnum::DEVICES_CTRL){
            if(_motors_references_queue.read_available()>0){
                
                while(_motors_references_queue.pop(_internal_motors_references))
                {}
         
                std::vector<MR> mot_ref_v;
                for ( const auto &[bId,motor_tx] : _internal_motors_references ) {
                    auto ctrl_type=std::get<0>(motor_tx);
                    if(ctrl_type!=0x00){
                        mot_ref_v.push_back(std::tuple_cat(std::make_tuple(bId),motor_tx));
                    }
                }
                
                if(!mot_ref_v.empty()){
                    MSR m_ref_flag=std::make_tuple(1, mot_ref_v); //RefFlags::FLAG_MULTI_REF
                    CBuffT<4096u> sendBuffer{};
                    auto sizet = proto.packReplRequestSetMotorsRefs(sendBuffer, m_ref_flag);
                    do_send(sendBuffer.data(), sendBuffer.size() );
                    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
                }
            }

        }
        else{
            _consoleLog->error("Cannot send references to the motors since not controlled, stop sending them");
        }
    }
    else{
        _consoleLog->error("Client in error state, please stop the main process!");
    }
}

