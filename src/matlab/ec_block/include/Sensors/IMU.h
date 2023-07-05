#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/ImuSensor.h>


namespace XBB_IMU{
    class XBotBlock_IMU;
}

class XBB_IMU::XBotBlock_IMU : public blockfactory::core::Block
{
private:
    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    XBot::ImuSensor::ConstPtr _IMU;
    
    std::string _manager_name,_manager_type;
    uint8_t _robot_id,_model_id;

    std::vector<std::string> _IMU_list;

    enum IMU_Components
    {
        orient,
        quat,
        a,
        omega,
    };
    
    std::map<std::string, IMU_Components> _IMU_option {
        { "orient", orient },
        { "quat", quat },
        { "a", a },
        { "omega", omega }
    };
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_IMU() = default;
    ~XBotBlock_IMU() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool setIMUOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

