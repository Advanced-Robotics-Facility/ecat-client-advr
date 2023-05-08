#ifndef EC_TYPES_H
#define EC_TYPES_H

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

enum ClientCmdType { STOP, START};

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

#endif // EC_TYPES_H