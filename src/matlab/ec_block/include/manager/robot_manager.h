#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>

#include "manager/reading.h"
#include "manager/reference.h"

namespace EcBlock{
    class RobotManager;
}

class EcBlock::RobotManager : public blockfactory::core::Block
{
private:
    EcIface::Ptr _robot;
    MotorStatusMap _motors_status_map;
    std::vector<MR> _motors_ref;
    std::vector<std::string> _readings_list,_references_list;
    std::shared_ptr<EcBlock::Reading> _readings_ptr;
    std::shared_ptr<EcBlock::Reference> _references_ptr;
    
    bool _do_move,_avoid_first_move;
    
    
public:
 
    static const std::string ClassName;
    
    
    RobotManager() = default;
    ~RobotManager() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    void robot_sensing();
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

