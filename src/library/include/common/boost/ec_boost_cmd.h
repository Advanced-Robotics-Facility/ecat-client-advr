#ifndef EC_BOOST_CMD_H
#define EC_BOOST_CMD_H

#include <boost/asio.hpp>

#include "ec_iface.h"
#include "task.h"
#include "pck_msgs.h"
#include <chrono>
using boost::asio::ip::udp;

class EcBoostCmd : public UdpTask<EcBoostCmd, MsgPackProtocol>, virtual public EcIface
{
public:

    EcBoostCmd(std::string task_name,std::string host_address,uint32_t host_port);
    ~EcBoostCmd();

    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client() = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;

    bool start_motors(const MST &) final;
    bool stop_motors() final;
    bool pdo_aux_cmd(const PAC & pac) final;
    bool retrieve_slaves_info(SSI &slave_info) final;
    bool retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo) final;
    bool retrieve_rr_sdo(uint32_t esc_id,
                         const RD_SDO &rd_sdo, 
                         const WR_SDO &wr_sdo,
                         RR_SDO &rr_sdo) final;
                         
    bool set_wr_sdo(uint32_t esc_id,
                    const RD_SDO &rd_sdo,
                    const WR_SDO &wr_sdo) final;

    void send_pdo();

private:

    //COMMANDS
    void get_slaves_info();
    void getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo);
    
    void feed_motors();
    void set_motors_gains(const MSG &);

    bool get_reply_from_server(ReplReqRep cmd_req);
    
    void set_wait_reply_time(uint32_t wait_reply_time);
    void restore_wait_reply_time();
    
    // MSG HANDLERS
    void repl_replies_handler(char*buf, size_t size);
    
    std::shared_ptr<std::condition_variable> _cv_repl_reply;
    
    RR_SDO _rr_sdo;
    
    const int _max_cmd_attemps=3;
    uint32_t _wait_reply_time;
    bool _cmd_req_reply;
    
protected:
    void connect();
    void disconnect();
    void quit_server();
    void ping(bool test);
    std::chrono::duration<int, std::milli>_server_alive_check_ms;

private:
    std::string _reply_err_msg;
    ReplReqRep _repl_req_rep;
    SSI _slave_info;
    int _client_port;
};

#endif
