#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/ModelInterface.h>

#include <memory>
#include <string>

namespace XBB_FloatingBase{
    class XBotBlock_FloatingBase;
}

class XBB_FloatingBase::XBotBlock_FloatingBase
{
private:

    XBot::ModelInterface::Ptr _model;
    
    std::string _model_name;
    
    Eigen::Affine3d _get_pose_frame,_set_pose_frame;
    Eigen::Matrix3d _orientation;
    Eigen::Vector6d _get_twist,_set_twist;
    Eigen::Vector3d _angular_velocity;
                
    
    std::vector<std::string> _get_floating_base_list;
    std::string _set_floating_base_component;
    
    size_t _in_start_port,_out_start_port;

    enum FloatingBase_Components
    {
        Pose,
        State,
        Orientation,
        Twist,
        AngularVelocity,
    };
    
    std::map<std::string, FloatingBase_Components> _floating_base_option {
        { "Pose", Pose },
        { "State", State },
        { "Orientation", Orientation },
        { "Twist", Twist },
        { "AngularVelocity", AngularVelocity }
    };
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_FloatingBase(XBot::ModelInterface::Ptr model,
                           std::vector<std::string> get_floating_base_list,
                           std::string set_floating_base_component,
                           size_t in_start_port,
                           size_t out_start_port);
    ~XBotBlock_FloatingBase(){};

    bool configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                               blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getFloatingBase(const blockfactory::core::BlockInformation* blockInfo);
    bool setFloatingBase(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

