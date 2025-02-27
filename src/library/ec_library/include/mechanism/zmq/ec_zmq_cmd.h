#ifndef EC_ZMQ_CMD_H
#define EC_ZMQ_CMD_H

#include "api/ec_iface.h"
#include "mechanism/zmq/ec_repl_cmd.h"

class EcZmqCmd : public virtual EcIface
{
public:

    EcZmqCmd(std::string protocol,std::string host_address,uint32_t host_port);
    ~EcZmqCmd();

    virtual void start_client(uint32_t period_ms) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;

    bool cmd_error_status(EcReplFault fault, std::string op, std::string &msg);
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
    void stop_cmd();

private:
    
    void feed_motors(); 
    void feed_valves();
    void feed_pumps();
    
    EcReplCmd::Ptr  _ec_repl_cmd;
    const int _max_cmd_attemps=3;
    SSI _slave_info;
    DST _devices_started;
};

#endif
