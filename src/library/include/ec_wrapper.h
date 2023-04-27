#ifndef EC_WRAPPER_H
#define EC_WRAPPER_H

#include <memory>
#include <array>
#include <vector>
#include <map>

using MotorStatusMap = std::map<int, std::tuple<float, float, float, float,   // pos_{link,motor}, vel_{link,motor}
                                                float,                        // tor_ref
                                                float,float,                  // {motor,board}
                                                uint32_t, uint32_t,           // fault, rtt, op_idx_ack                  
                                                uint32_t, float, uint32_t>>;  // aux // cmd_aux_sts
using FtStatusMap = std::map<int, std::vector<float>>;
using PwrStatusMap = std::map<int32_t, std::vector<float>>;

// MotorsRef
using MR = std::tuple<int32_t, int32_t,                     // bId, ctrl_type
                      float, float, float,                  // pos_ref, vel_ref, tor_ref
                      float, float, float, float, float,    // gains
                      uint32_t, uint32_t, float>;           // op, idx, aux
                      
// MotorsStarT
using MST = std::vector<std::tuple<int32_t, int32_t, std::vector<float>>>;

// Sdo commands
using RD_SDO = std::vector<std::string>;
using WR_SDO = std::vector<std::tuple<std::string, std::string>>;
using RR_SDO = std::map<std::string, float>;

// PdoAuxCommand
using PAC = std::vector<std::tuple<int32_t, int32_t>>;

// SlaveSInfo
using SSI = std::vector<std::tuple<int32_t, int32_t, int32_t>>;

enum class MotorRefFlags : uint32_t
{
    FLAG_NONE           = 0x0,        //
    FLAG_MULTI_REF      = 1 << 0,
    FLAG_LAST_REF       = 1 << 1,
};


class EcWrapper
{
public:
    
    typedef std::shared_ptr<EcWrapper> Ptr;
    virtual ~EcWrapper() {};

    // EtherCAT Client ADVR Facilty manager
    virtual void connect(void) = 0;
    virtual void disconnect(void) = 0;
    //virtual void periodicActivity(void) = 0;
    virtual void stop_client(void) = 0;
    virtual bool is_client_alive(void) = 0;
    
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

#endif // EC_WRAPPER_H
