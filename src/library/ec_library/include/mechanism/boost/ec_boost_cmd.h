#ifndef EC_BOOST_CMD_H
#define EC_BOOST_CMD_H

#include "api/ec_iface.h"
#include "ec_boost.h"

class EcBoostCmd : virtual public EcBoost, virtual public EcIface
{
public:

    EcBoostCmd();
    ~EcBoostCmd();

    virtual void start_client(uint32_t period_ms) = 0;
    virtual void stop_client() = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;

    bool start_devices(const DST &) final;
    bool stop_devices() final;
    bool pdo_aux_cmd(const PAC & pac) final;
    bool retrieve_slaves_info(SSI &slave_info) final;
    bool retrieve_all_sdo(uint32_t esc_id,RR_SDOS &rr_sdo) final;
    bool retrieve_rr_sdo(uint32_t esc_id,
                         const RD_SDO &rd_sdo, 
                         const WR_SDO &wr_sdo,
                         RR_SDOS &rr_sdo) final;
                         
    bool set_wr_sdo(uint32_t esc_id,
                    const RD_SDO &rd_sdo,
                    const WR_SDO &wr_sdo) final;

    void send_pdo();

private:
    
    // MSG HANDLERS
    void server_replies_handler(char*buf, size_t size);
    void repl_replies_handler(char*buf, size_t size);

    //COMMANDS
    void get_slaves_info();
    void getAndset_slaves_sdo(uint32_t esc_id, const RD_SDO &rd_sdo, const WR_SDO &wr_sdo);
    
    void feed_motors();
    void set_motors_gains(const MSG &);

    bool get_reply_from_server(ReplReqRep cmd_req);
    
    void set_wait_reply_time(uint32_t wait_reply_time);
    void restore_wait_reply_time();
    
    std::shared_ptr<std::condition_variable> _cv_repl_reply;
    
    std::map<std::string,std::string> _rr_sdo;
    std::vector<std::string> _sdo_names;
    
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
};

#endif
