#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>

#include "manager/reading.h"
#include "manager/reference.h"

namespace EcBlock{
    class EcManager;
}

class EcBlock::EcManager : public blockfactory::core::Block
{
private:
    MotorStatusMap _motors_status_map;
    FtStatusMap _ft6_status_map;
    ImuStatusMap _imu_status_map;
    PwrStatusMap _pow_status_map;
    
    
    std::vector<MR> _motors_ref;
    std::vector<std::string> _readings_list,_references_list;
    std::shared_ptr<EcBlock::Reading> _readings_ptr;
    std::shared_ptr<EcBlock::Reference> _references_ptr;
    
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

