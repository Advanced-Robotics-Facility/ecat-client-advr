#ifndef EC_UDP_H
#define EC_UDP_H


#include <chrono>
#include <boost/asio.hpp>

#include "ec_iface.h"
#include "task.h"
#include "pck_msgs.h"
using boost::asio::ip::udp;

/////////////////////////////////////////////////////

constexpr static int CLIENT_PORT{54320};

/**
 * @brief The ReplServer class
 */
class EcUDP : public UdpTask<EcUDP, MsgPackProtocol>, public EcIface
{
public:

    EcUDP(std::string host_address,uint32_t host_port);
    ~EcUDP();

    void start_client(uint32_t period_ms,bool logging) final;
    void stop_client() final ;
    void set_loop_time(uint32_t period_ms) final;
    
    void receive_error(std::error_code ec);
    void periodicActivity();

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



private:

    //COMMANDS
    void connect();
    void disconnect();
    void quit_server();
    void ping(bool test);
    
    void get_slaves_info();
    void getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo);
    
    void feed_motors(const MSR &);
    void set_motors_gains(const MSG &);

    bool get_reply_from_server(ReplReqRep cmd_req);
    
    void set_wait_reply_time(uint32_t wait_reply_time);
    void restore_wait_reply_time();
    
    // MSG HANDLERS
    void server_replies_handler(char*buf, size_t size);
    void repl_replies_handler(char*buf, size_t size);
    void server_status_handler(char*buf, size_t size);
    void motor_status_handler(char*buf, size_t size);
    void ft6_status_handler(char*buf, size_t size);
    void pwr_status_handler(char*buf, size_t size);

    
    std::shared_ptr<std::condition_variable> _cv_repl_reply;
    
    RR_SDO _rr_sdo;
    
    const int _max_cmd_attemps=3;
    uint32_t _wait_reply_time;
    std::chrono::steady_clock::time_point _client_alive_time; 
    std::chrono::duration<int, std::milli>_server_alive_check_ms;
    bool _cmd_req_reply;
    ClientStatus _client_status;

private:
    std::string _reply_err_msg;
    ReplReqRep _repl_req_rep;
    ServerStatus _actual_server_status;

    std::shared_ptr<std::thread> _ec_udp_thread;
    
    SSI _slave_info;
};

#endif // EC_UDP_H
