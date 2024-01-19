#ifndef EC_CMD_H
#define EC_CMD_H

#include "ec_iface.h"
#include "protocols/common/zmq/ec_zmq_cmd.h"
#include "cmn_utils.h"

class EcCmd : public EcIface
{
public:

    EcCmd(std::string protocol,std::string host_address,uint32_t host_port);
    ~EcCmd();

    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client(void) = 0;
    virtual bool is_client_alive(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;
    
    // EtherCAT Client ADVR Facilty logger
    virtual void start_logging(void) = 0;
    virtual void stop_logging(void) = 0;
    
    // EtherCAT Client ADVR Facilty getters
    virtual MotorStatusMap get_motors_status(void) = 0;
    virtual FtStatusMap get_ft6_status(void) = 0;
    virtual PwrStatusMap get_pow_status(void) = 0;
    virtual ImuStatusMap get_imu_status(void) = 0;
    virtual bool pdo_aux_cmd_sts(const PAC & pac) = 0;
    
    // EtherCAT Client ADVR Facilty setters
    virtual void set_motors_references(const MotorRefFlags, const std::vector<MR>) = 0;
    
    bool cmd_error_status(EcZmqFault fault, std::string op, std::string &msg);
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
                    
    void feed_motors(std::vector<MR> motors_references);
    
    bool client_sts();
    

private:
    EcZmqCmd::Ptr  _ec_zmq_cmd;
    SSI _slave_info;
    
    bool _client_alive;
    const int _max_cmd_attemps=3;
    std::shared_ptr<spdlog::logger> _consoleLog;
    
};

#endif // EC_CMD_H
