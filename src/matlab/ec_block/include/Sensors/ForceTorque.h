#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/ImuSensor.h>


namespace XBB_ForceTorque{
    class XBotBlock_ForceTorque;
}

class XBB_ForceTorque::XBotBlock_ForceTorque : public blockfactory::core::Block
{
private:
    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    XBot::ForceTorqueSensor::ConstPtr _force_torque;
    
    std::string _manager_name,_manager_type;
    uint8_t _robot_id,_model_id;

    std::vector<std::string> _force_torque_list;

    enum ForceTorque_Components
    {
        f,
        t,
        w,
    };
    
    std::map<std::string, ForceTorque_Components> _force_torque_option {
        { "f", f },
        { "t", t },
        { "w", w }
    };
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_ForceTorque() = default;
    ~XBotBlock_ForceTorque() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool setForceTorqueOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

