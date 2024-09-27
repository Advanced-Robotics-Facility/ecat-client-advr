#ifndef EC_TYPES_H
#define EC_TYPES_H

#include <map>
#include <vector>
#include <tuple>

#include <esc_info.h>
#include "mechanism/protobuf/motor/hhcm/hhcm_pdo.h"
#include "mechanism/protobuf/motor/circulo9/circulo9_pdo.h"
#include "mechanism/protobuf/motor/flexpro/flexpro_pdo.h"
#include "mechanism/protobuf/imu/imu_pdo.h"
#include "mechanism/protobuf/ft/ft_pdo.h"
#include "mechanism/protobuf/pow/pow_pdo.h"
#include "mechanism/protobuf/valve/valve_pdo.h"
#include "mechanism/protobuf/pump/pump_pdo.h"

static std::map<uint32_t,std::string>ec_motors={{iit::ecat::CENT_AC,"HHCM_HP_Motor"},
                                                {iit::ecat::LO_PWR_DC_MC,"HHCM_LP_Motor"},
                                                {iit::ecat::SYNAPTICON_v5_0,"Synapticon_Motor"},
                                                {iit::ecat::SYNAPTICON_v5_1,"Synapticon_Motor"},
                                                {iit::ecat::AMC_FLEXPRO,"AMC_flex_pro_Motor"}};
                                            
using MotorStatusMap =   std::map<int32_t, MotorPdoRx::pdo_t>;
using MotorReferenceMap= std::map<int32_t, MotorPdoTx::pdo_t>; 
using PwrStatusMap=      std::map<int32_t, PowPdoRx::pdo_t>;
using FtStatusMap=       std::map<int32_t, FtPdoRx::pdo_t>;
using ImuStatusMap=      std::map<int32_t, ImuPdoRx::pdo_t>;
using ValveStatusMap=    std::map<int32_t, ValvePdoRx::pdo_t>;
using ValveReferenceMap= std::map<int32_t, ValvePdoTx::pdo_t>;    
using PumpStatusMap =    std::map<int32_t, PumpPdoRx::pdo_t>;
using PumpReferenceMap=  std::map<int32_t, PumpPdoTx::pdo_t >;     
                      
// MotorsStarT
using MST = std::vector<std::tuple<int32_t, int32_t, std::vector<float>>>;

// Sdo commands
using RD_SDO = std::vector<std::string>;
using WR_SDO = std::vector<std::tuple<std::string, std::string>>;
using RR_SDO = std::map<std::string, float>;
using SRD_SDO = std::map<uint32_t, RR_SDO>;

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


#endif // EC_TYPES_H
