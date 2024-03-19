#ifndef EC_ZMQ_CMD_H
#define EC_ZMQ_CMD_H

#include "ec_iface.h"
#include "common/zmq/ec_repl_cmd.h"

class EcZmqCmd : public virtual EcIface
{
public:

    EcZmqCmd(std::string protocol,std::string host_address,uint32_t host_port);
    ~EcZmqCmd();

    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;

    bool cmd_error_status(EcReplFault fault, std::string op, std::string &msg);
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
    
    void feed_motors(); 
    void feed_valves();
    void feed_pumps();
    
    EcReplCmd::Ptr  _ec_repl_cmd;
    const int _max_cmd_attemps=3;
    SSI _slave_info;

    MotorReferenceMap _motors_references_cmd;
    ValveReferenceMap _valves_references_cmd;
    PumpReferenceMap _pumps_references_cmd;
};

#endif
