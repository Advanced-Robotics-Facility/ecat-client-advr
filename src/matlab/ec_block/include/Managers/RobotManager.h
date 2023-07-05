#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>
#include <XBotInterface/ModelInterface.h>

#include "Managers/Readings.h"
#include "Managers/References.h"

namespace XBB_RobotManager{
    class XBotBlock_RobotManager;
}

class XBB_RobotManager::XBotBlock_RobotManager : public blockfactory::core::Block
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

    
    std::string _robot_name,_model_name;
    std::vector<std::string> _readings_list,_references_list;
    std::shared_ptr<XBB_Readings::XBotBlock_Readings> _readings_ptr;
    std::shared_ptr<XBB_References::XBotBlock_References> _references_ptr;

    int _n_joint;
    
    bool _do_sense, _do_move,_avoid_first_move;
    
    
public:
 
    static const std::string ClassName;
    
    
    XBotBlock_RobotManager() = default;
    ~XBotBlock_RobotManager() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool robot_sensing();
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

