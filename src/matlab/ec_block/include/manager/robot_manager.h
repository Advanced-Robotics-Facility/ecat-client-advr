#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>
#include <XBotInterface/ModelInterface.h>

#include "manager/reading.h"
#include "manager/reference.h"

namespace EcBlock{
    class RobotManager;
}

class EcBlock::RobotManager : public blockfactory::core::Block
{
private:
    EcIface::Ptr _robot_new;
    XBot::RobotInterface::Ptr _robot;
    
    enum class UseExtRobot
    {
        OFF,
        ON
    };
    
    UseExtRobot _use_ext_robot;

    std::vector<std::string> _readings_list,_references_list;
    std::shared_ptr<EcBlock::Reading> _readings_ptr;
    std::shared_ptr<EcBlock::Reference> _references_ptr;
    
    bool _do_sense, _do_move,_avoid_first_move;
    
    
public:
 
    static const std::string ClassName;
    
    
    RobotManager() = default;
    ~RobotManager() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool robot_sensing();
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

