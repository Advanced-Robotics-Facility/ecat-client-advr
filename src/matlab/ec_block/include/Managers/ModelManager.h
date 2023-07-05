#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <memory>
#include <string>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>

#include "XBotBlock/Managers/FloatingBase.h"
#include "XBotBlock/Managers/Gravity.h"
#include "XBotBlock/Managers/MechanicalLimits.h"
#include "XBotBlock/Managers/Readings.h"
#include "XBotBlock/Managers/References.h"
#include "XBotBlock/Managers/SensorManager.h"


namespace XBB_ModelManager{
    class XBotBlock_ModelManager;
}

class XBB_ModelManager::XBotBlock_ModelManager : public blockfactory::core::Block
{
private:
    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    
    enum class UseExtModel
    {
        OFF,
        ON
    };
    
    UseExtModel _use_ext_model;
    
    
    enum class SyncWithRobot
    {
        OFF,
        ON
    };

    SyncWithRobot _sync_with_robot;
    
    std::vector<std::string> _get_floating_base_list;
    std::string _set_floating_base_component;
    std::string _gravity_request,_reference_frame;
    
    bool _limits_check,_limits_enforce;
    
    
    Eigen::Affine3d _pose_frame;
    
    std::string _robot_name,_model_name;
    uint8_t _robot_id,_model_id;
    std::vector<std::string> _readings_list,_mechanical_limits_list,_sensor_list,_references_list;
    std::shared_ptr<XBB_FloatingBase::XBotBlock_FloatingBase> _floating_base_ptr;
    std::shared_ptr<XBB_Gravity::XBotBlock_Gravity> _gravity_ptr;
    std::shared_ptr<XBB_Readings::XBotBlock_Readings> _readings_ptr;
    std::shared_ptr<XBB_MechanicalLimits::XBotBlock_MechanicalLimits> _mechanical_limits_ptr;
    std::shared_ptr<XBB_References::XBotBlock_References> _references_ptr;
    std::shared_ptr<XBB_SensorManager::XBotBlock_SensorManager> _sensor_manager_ptr;
    
public:
 
    static const std::string ClassName;
    
    
    XBotBlock_ModelManager() = default;
    ~XBotBlock_ModelManager() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool model_updating();
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

