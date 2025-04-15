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
                /*for ( auto &[k,v] : _rr_sdo ) {
                    _consoleLog->info( " <-- id {} : {} = {}", esc_id, k, v);
                }*/
        }break;
        case ReplReqRep::SDO_INFO:{
            uint32_t esc_id;
            _sdo_names.clear();
            ret = proto.getReplReplySdoNames(buf,size, esc_id, _sdo_names,_reply_err_msg);
            /*for ( auto sdo : _sdo_names ) {
                _consoleLog->info( " <-- id {} : {} = {}", esc_id,sdo);
            }*/
    }break;

        default:
            break;
    }
    
    if(_cmd_req_reply){
        if(_reply_err_msg.find("RECV_FAIL")!=std::string::npos){
            _req_reply_timeout=true; //repl timeout
             _consoleLog->error("Repl replies timeout, restart the request!");
        }
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
    auto cv_status= _cv_repl_reply->wait_for(lk,std::chrono::milliseconds(_wait_reply_time)); // timer for ACK and NACK from udp server
    if(cv_status==std::cv_status::timeout){
        _req_reply_timeout=true;
        _consoleLog->error("Command timeout, restart the request!"); // server timeout 
    }
    else{
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE && !_req_reply_timeout){
            _consoleLog->info(" Command requested ---> {} Command reply--->{} ", cmd_req, _repl_req_rep);
            if(cmd_req == _repl_req_rep){
                if(_reply_err_msg == "OkI"){
                    _reply_err_msg = "";
                    return true;
                }
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
    _req_reply_timeout=false;
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
                if(_req_reply_timeout){
                    return false;
                }
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }
    
    if(_slave_info.empty()){
        if(!_fake_slave_info.empty() && 
            _client_status.status!=ClientStatusEnum::NOT_ALIVE){
            _slave_info=_fake_slave_info;
            slave_info = _slave_info;
            ret_cmd_status=true;
        }     
    }

    return ret_cmd_status;
}

void EcBoostCmd::getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo)
{
    CBuffT<8192u> sendBuffer{};
    
    RDWR_SDO rdwr_sdo = std::make_tuple(esc_id,rd_sdo,wr_sdo);

    auto sizet = proto.packReplRequestSdoCmd (sendBuffer, rdwr_sdo);
    do_send(sendBuffer.data(), sendBuffer.size() );
    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
}

bool EcBoostCmd::retrieve_all_sdo(uint32_t esc_id,RR_SDOS &rr_sdo)
{
    int attemps_cnt = 0;
    bool ret_cmd_status=false;
    
    _req_reply_timeout=false;
    while(rr_sdo.empty() && attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            CBuffT<8192u> sendBuffer{};

            auto sizet = proto.packReplRequestSdoNames (sendBuffer, esc_id);
            do_send(sendBuffer.data(), sendBuffer.size() );
            _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);

            ret_cmd_status = get_reply_from_server(ReplReqRep::SDO_INFO);
    
            if(ret_cmd_status){
                attemps_cnt = _max_cmd_attemps;
                ret_cmd_status = retrieve_rr_sdo(esc_id,_sdo_names,{},rr_sdo);
            }
            else{
                attemps_cnt++;
                if(_req_reply_timeout){
                    return false;
                }
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return false;
        }
    }

    return ret_cmd_status;
}

int EcBoostCmd::sdo_cmd(uint32_t esc_id, 
                        const RD_SDO &rd_sdo, 
                        const WR_SDO &wr_sdo)
{
    int attemps_cnt = 0;
    _req_reply_timeout=false;

    while(attemps_cnt < _max_cmd_attemps){
        if(_client_status.status!=ClientStatusEnum::NOT_ALIVE){
            getAndset_slaves_sdo(esc_id,rd_sdo,wr_sdo);

            if(get_reply_from_server(ReplReqRep::SDO_CMD)){
                return 1;
            }
            else{
                attemps_cnt++;
                if(_req_reply_timeout){
                    return -1;
                }
            }
        }
        else{
            _consoleLog->error("Client in not alive state, please stop the main process!");
            return -1;
        }
    }
    return 0;
}

bool EcBoostCmd::retrieve_rr_sdo(uint32_t esc_id,
                                 const RD_SDO &rd_sdo, 
                                 const WR_SDO &wr_sdo,
                                 RR_SDOS &rr_sdo)

{
    RD_SDO rd_sdo_chunk;
    size_t count_sdo_chunk=0;
    size_t count_sdo=0;
    const size_t max_chunk_sdo=5;
    for(const auto &sdo_name:rd_sdo){
        if(count_sdo_chunk<max_chunk_sdo-1 && count_sdo<rd_sdo.size()-1){
            rd_sdo_chunk.push_back(sdo_name);
            count_sdo_chunk++;
        }
        else{
            rd_sdo_chunk.push_back(sdo_name); // get last element
            
            _rr_sdo.clear();
            if(sdo_cmd(esc_id,rd_sdo_chunk,{})<0){
                return false;
            }

            for(const auto&[sdo_name,sdo_value]:_rr_sdo){
                rr_sdo[sdo_name]=sdo_value;
            }

            std::string sdo_name_error="";
            for(const auto &sdo_name_chunk:rd_sdo_chunk){
                if(rr_sdo.count(sdo_name_chunk)==0){
                    sdo_name_error=sdo_name_error+" "+sdo_name_chunk;
                }
            }

            if(sdo_name_error!=""){
                _consoleLog->error("Error on read the SDO(s): {}",sdo_name_error);
            }
            
            count_sdo_chunk=0;
            rd_sdo_chunk.clear();
        }
        count_sdo++;
    }

    return true;
}

bool EcBoostCmd::set_wr_sdo(uint32_t esc_id,
                        const RD_SDO &rd_sdo,
                        const WR_SDO &wr_sdo)

{
    WR_SDO wr_sdo_chunk;
    size_t count_sdo_chunk=0;
    size_t count_sdo=0;
    const size_t max_chunk_sdo=5;
    for(const auto &sdo_tuple:wr_sdo){
        if(count_sdo_chunk<max_chunk_sdo-1 && count_sdo<wr_sdo.size()-1){
            wr_sdo_chunk.push_back(sdo_tuple);
            count_sdo_chunk++;
        }
        else{
            wr_sdo_chunk.push_back(sdo_tuple); // get last element

            int ret = sdo_cmd(esc_id,{},wr_sdo_chunk);
            if(ret<0){
                return false;
            }
            else if(ret==0){
                std::string sdo_name_error="";
                for(const auto &[sdo_name_chunk ,value]:wr_sdo_chunk){
                    sdo_name_error=sdo_name_error+" "+sdo_name_chunk;
                }
                _consoleLog->error("Error on write the SDO(s): {}",sdo_name_error);
            }
            count_sdo_chunk=0;
            wr_sdo_chunk.clear();
        }
        count_sdo++;
    }

    return true;
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
        _req_reply_timeout=false;
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
                    restore_wait_reply_time();
                    if(_req_reply_timeout){
                        return false;
                    }
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
    _req_reply_timeout=false;
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
                if(_req_reply_timeout){
                    return false;
                }
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

                std::vector<MR> mot_ref_v;
                for ( const auto &[bId,motor_tx] : _motor_reference_map ) {
                    auto ctrl_type=std::get<0>(motor_tx);
                    if(ctrl_type!=0x00){
                        mot_ref_v.push_back(std::tuple_cat(std::make_tuple(bId),motor_tx));
                    }
                }
                
                if(!mot_ref_v.empty()){
                    MSR m_ref_flag=std::make_tuple(_reference_flag, mot_ref_v); //RefFlags::FLAG_MULTI_REF
                    CBuffT<4096u> sendBuffer{};
                    auto sizet = proto.packReplRequestSetMotorsRefs(sendBuffer, m_ref_flag);
                    do_send(sendBuffer.data(), sendBuffer.size() );
                    _consoleLog->info(" --{}--> {} ", sizet, __FUNCTION__);
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

