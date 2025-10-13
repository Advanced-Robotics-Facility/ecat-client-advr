#ifndef EC_TYPES_H
#define EC_TYPES_H

#include <map>
#include <vector>
#include <tuple>

#include <esc_info.h>
#include "mechanism/protobuf/motor/advrf/advrf_pdo.h"
#include "mechanism/protobuf/motor/synapticon/synapticon_pdo.h"
#include "mechanism/protobuf/imu/imu_pdo.h"
#include "mechanism/protobuf/ft/ft_pdo.h"
#include "mechanism/protobuf/pow/pow_pdo.h"
#include "mechanism/protobuf/valve/valve_pdo.h"
#include "mechanism/protobuf/pump/pump_pdo.h"

static std::map<uint32_t,std::string>ec_motors = [] {
    std::map<uint32_t, std::string> result;
    for (const auto& [key, value] : iit::ecat::esc_type_map) {
        if ((value == "Motor" || value.size() > 6) && (value.substr(value.size() - 6) == "_Motor")) {
            std::string motor=value;
            if(motor=="Motor"){
                motor="ADVRF_Motor";
            }
            result[key] = value;
        }
    }
    return result;
}();

static std::map<uint32_t,std::string>ec_valves= [] {
    std::map<uint32_t, std::string> result;
    for (const auto& [key, value] : iit::ecat::esc_type_map) {
        if ((value == "Valve" || value.size() > 6) && (value.substr(value.size() - 6) == "_Valve")) {
            result[key] = value;
        }
    }
    return result;
}();   

static std::map<uint32_t,std::string>ec_pumps= [] {
    std::map<uint32_t, std::string> result;
    for (const auto& [key, value] : iit::ecat::esc_type_map) {
        if ((value == "Hpu" || value.size() > 6) && (value.substr(value.size() - 6) == "_Hpu")) {
            result[key] = value;
        }
    }
    return result;
}();   

using MotorStatusMap =   std::map<int32_t, MotorPdoRx::pdo_t>;
using MotorReferenceMap= std::map<int32_t, MotorPdoTx::pdo_t>; 
using PwrStatusMap=      std::map<int32_t, PowPdoRx::pdo_t>;
using FtStatusMap=       std::map<int32_t, FtPdoRx::pdo_t>;
using ImuStatusMap=      std::map<int32_t, ImuPdoRx::pdo_t>;
using ValveStatusMap=    std::map<int32_t, ValvePdoRx::pdo_t>;
using ValveReferenceMap= std::map<int32_t, ValvePdoTx::pdo_t>;    
using PumpStatusMap =    std::map<int32_t, PumpPdoRx::pdo_t>;
using PumpReferenceMap=  std::map<int32_t, PumpPdoTx::pdo_t >;     
                      
// DevicesStarT
using DST = std::vector<std::tuple<int32_t, int32_t, std::vector<float>>>;

// Sdo commands
using RD_SDO = std::vector<std::string>;
using WR_SDO = std::vector<std::tuple<std::string, std::string>>;
using RR_SDOS = std::map<std::string, std::string>;
using SRD_SDO = std::map<uint32_t, RR_SDOS>;

// PdoAuxCommand
using PAC = std::vector<std::tuple<int32_t, int32_t>>;

// SlaveSInfo
using SSI = std::vector<std::tuple<int32_t, int32_t, int32_t>>;

enum class RefFlags : uint32_t
{
    FLAG_NONE           = 0x0,        //
    FLAG_MULTI_REF      = 1 << 0,
    FLAG_LAST_REF       = 1 << 1,
};

enum ClientCmdType { STOP, START};

enum class ClientStatusEnum : uint32_t
{
    ERROR            = 0,        // Error
    NOT_ALIVE        = 1 << 0,
    IDLE             = 1 << 1,   
    CONNECTED        = 1 << 2,   
    DEVICES_MAPPED   = 1 << 3,   
    DEVICES_STARTED  = 1 << 4,   
    DEVICES_CTRL     = 1 << 5,   
    DEVICES_STOPPED  = 1 << 6,   
};


template <typename E>
constexpr auto to_underlying_enum(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}


enum class PdoAuxCmdTypeEnum : int32_t
{
    BRAKE_RELEASE   = 1,
    BRAKE_ENGAGE,
    LED_ON,
    LED_OFF,
};

enum DeviceCtrlType{
    MOTOR   = 0,
    VALVE   = 1,
    PUMP    = 2,
};



#endif // EC_TYPES_H
