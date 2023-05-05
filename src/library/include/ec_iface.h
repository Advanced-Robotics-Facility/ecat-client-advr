#ifndef EC_IFACE_H
#define EC_IFACE_H

#include <memory>
#include <array>
#include <vector>
#include <map>
#include "ec_types.h"

class EcIface
{
public:
    
    typedef std::shared_ptr<EcIface> Ptr;
    virtual ~EcIface() {};

    // EtherCAT Client ADVR Facilty manager
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
    virtual bool pdo_aux_cmd_sts(const PAC & pac) = 0;
    
    // EtherCAT Client ADVR Facilty setters
    virtual void set_motors_references(const MotorRefFlags &, const std::vector<MR> &) = 0;
    
    // EtherCAT Client ADVR Facilty commands
    virtual bool start_motors(const MST &) = 0;
    virtual bool stop_motors(void) = 0;
    virtual bool pdo_aux_cmd(const PAC & pac) = 0;

    virtual bool retrieve_slaves_info(SSI &slave_info) = 0;
    virtual bool retrieve_rr_sdo(uint32_t esc_id,
                                 const RD_SDO &rd_sdo, 
                                 const WR_SDO &wr_sdo,
                                 RR_SDO &rr_sdo) = 0;
    virtual bool set_wr_sdo(uint32_t esc_id,
                            const RD_SDO &rd_sdo,
                            const WR_SDO &wr_sdo) = 0;
};

#endif // EC_IFACE_H
