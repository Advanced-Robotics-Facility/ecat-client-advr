#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>

#include "XBotBlock/Utilities/Pose.h"
#include "XBotBlock/Utilities/Twist.h"
#include "XBotBlock/Utilities/COM.h"


namespace XBB_Kinematics{
    class XBotBlock_Kinematics;
}

class XBB_Kinematics::XBotBlock_Kinematics : public blockfactory::core::Block
{
private:

    XBot::ModelInterface::Ptr _model;
    std::string _model_name;
    std::string _target_frame,_source_frame;
    std::string _link_name,_base_link_name;
    std::string _reference_frame;
                                        
    uint8_t _model_id;
    
    
    std::vector<std::string> _pose_list;
    std::vector<std::string> _twist_list;
    std::vector<std::string> _COM_list;
    
    std::shared_ptr<XBB_Pose::XBotBlock_Pose> _pose_ptr;
    std::shared_ptr<XBB_Twist::XBotBlock_Twist> _twist_ptr;
    std::shared_ptr<XBB_COM::XBotBlock_COM> _COM_ptr;
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_Kinematics() = default;
    ~XBotBlock_Kinematics() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

