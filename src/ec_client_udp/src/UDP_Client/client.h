#ifndef CLIENT_UDP_H
#define CLIENT_UDP_H

#include <memory>
#include <array>
#include <vector>
#include <chrono>
#include <boost/asio.hpp>

#include "cmn_utils.h"
#include "task.h"
#include "pck_msgs.h"

#include <mutex>


using boost::asio::ip::udp;

using MotorStatusMap = std::map<int, std::tuple<float, float, float, float,   // pos_{link,motor}, vel_{link,motor}
                                                float,                        // tor_ref
                                                float,float,                  // {motor,board}
                                                uint32_t, uint32_t,           // fault, rtt, op_idx_ack                  
                                                uint32_t, float, uint32_t>>;            // aux // cmd_aux_sts
using FtStatusMap = std::map<int, std::vector<float>>;
using PwrStatusMap = std::map<int32_t, std::vector<float>>;

enum class ClientStatus : uint32_t
{
    ERROR           = 0,        // Error
    WAITING_REPLY   = 1 << 0,
    IDLE            = 1 << 1,   
    CONNECTED       = 1 << 2,   
    MOTORS_MAPPED   = 1 << 3,   
    MOTORS_READY    = 1 << 4,   
    MOTORS_STARTED  = 1 << 5,   
    MOTORS_BRK_OFF  = 1 << 6,   
    MOTORS_CTRL     = 1 << 7,   
    MOTORS_STOPPED  = 1 << 8,   
    MOTORS_BRK_ON   = 1 << 9,   
    
};


/////////////////////////////////////////////////////

constexpr static int CLIENT_PORT{54320};

/**
 * @brief The ReplServer class
 */
class Client : public UdpTask<Client, MsgPackProtocol>
{
public:
    enum ClientCmdType { STOP, START};
    
    Client(std::string host_address,uint32_t host_port);

    void receive_error(std::error_code ec);
    void periodicActivity();

    void connect();
    void disconnect();
    void ping(bool test);
    void get_slaves_info();
    void getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo);
    MotorStatusMap get_motors_status();
    FtStatusMap get_ft6_status();
    PwrStatusMap get_pow_status();
    bool start_motors(const MST &);
    bool stop_motors();
    bool pdo_aux_cmd(const PAC & pac);
    bool pdo_aux_cmd_sts(const PAC & pac);
    void feed_motors(const MSR &);
    void set_motors_gains(const MSG &);
    void stop_client();
    bool get_reply_from_server(ReplReqRep cmd_req);
    bool retrieve_slaves_info(SSI &slave_info);
    bool retrieve_rr_sdo(uint32_t esc_id,
                         const RD_SDO &rd_sdo, 
                         const WR_SDO &wr_sdo,
                         RR_SDO &rr_sdo);
    bool is_client_alive();

private:

    // MSG HANDLERS
    void quit_server();
    void server_replies_handler(char*buf, size_t size);
    void repl_replies_handler(char*buf, size_t size);
    void server_status_handler(char*buf, size_t size);
    void motor_status_handler(char*buf, size_t size);
    void ft6_status_handler(char*buf, size_t size);
    void pwr_status_handler(char*buf, size_t size);
    void set_wait_reply_time(uint32_t wait_reply_time);
    void restore_wait_reply_time();
    std::shared_ptr<std::mutex> _mutex_motor_status;
    std::shared_ptr<std::mutex> _mutex_ft6_status;
    std::shared_ptr<std::mutex> _mutex_pow_status;
    std::shared_ptr<std::condition_variable> _cv_repl_reply;
    
    SSI _slave_info;
    RR_SDO _rr_sdo;
    
    const int _max_cmd_attemps=3;
    uint32_t _wait_reply_time;
    std::chrono::steady_clock::time_point _client_alive_time; 
    std::chrono::duration<int, std::milli>_server_alive_check_ms;
    bool _client_alive,_cmd_req_reply;
    ClientStatus _client_status;

private:
    std::shared_ptr<spdlog::logger> consoleLog;

    // last received motor data
    MotorStatusMap _motor_status_map;
    // last received ft data
    FtStatusMap _ft_status_map;
    // last received pow data
    PwrStatusMap _pow_status_map;
    
    std::string _reply_err_msg;
    ReplReqRep _repl_req_rep;
    ServerStatus _actual_server_status;
};

#endif // REPL_SERVER_H
