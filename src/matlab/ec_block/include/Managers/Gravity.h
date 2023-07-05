#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>

namespace XBB_Gravity{
    class XBotBlock_Gravity;
}

class XBB_Gravity::XBotBlock_Gravity
{
private:

    XBot::ModelInterface::Ptr _model;
    std::string _model_name,_gravity_request,_reference_frame;
    
    Eigen::Vector3d _get_gravity,_set_gravity;
                                        
    uint8_t _model_id;
    
    size_t _in_start_port,_out_start_port;
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_Gravity(XBot::ModelInterface::Ptr model,
                      std::string gravity_request,
                      std::string reference_frame,
                      size_t in_start_port,
                      size_t out_start_port);
    
    ~XBotBlock_Gravity(){};

    bool configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                               blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool gravity(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

