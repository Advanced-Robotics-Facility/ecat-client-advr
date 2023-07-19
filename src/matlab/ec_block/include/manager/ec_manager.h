#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>

#include "manager/ec_reading.h"
#include "manager/ec_reference.h"
#include "sensor/ec_imu.h"
#include "sensor/ec_ft.h"
#include "power/ec_pow.h"

namespace EcBlock{
    class EcManager;
}

class EcBlock::EcManager : public blockfactory::core::Block
{
private:
    MotorStatusMap _motors_status_map;
    FtStatusMap _ft_status_map;
    ImuStatusMap _imu_status_map;
    PwrStatusMap _pow_status_map;
    
    
    std::vector<MR> _motors_ref;
    std::vector<std::string> _readings_list,_references_list,_imu_list,_ft_list,_pow_list;
    std::shared_ptr<EcBlock::Reading> _readings_ptr;
    std::shared_ptr<EcBlock::Reference> _references_ptr;
    std::shared_ptr<EcBlock::Imu> _imu_ptr;
    std::shared_ptr<EcBlock::Ft> _ft_ptr;
    std::shared_ptr<EcBlock::Pow> _pow_ptr;
    
    bool _do_move,_avoid_first_move;
    
    
public:
 
    static const std::string ClassName;
    
    
    EcManager() = default;
    ~EcManager() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

