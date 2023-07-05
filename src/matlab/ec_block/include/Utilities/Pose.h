#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>


namespace XBB_Pose{
    class XBotBlock_Pose;
}

class XBB_Pose::XBotBlock_Pose
{
private:

    XBot::ModelInterface::Ptr _model;
    std::string _target_frame,_source_frame;
    std::string _link_name;
    
    Eigen::Affine3d _pose_frame;
    Eigen::Matrix3d _orientation;
    Eigen::Vector3d _source_point,_point_pos,_point,_point_acc;
                                        
    uint8_t _model_id;

    std::vector<std::string> _pose_list;
    
     size_t _in_start_port,_out_start_port;

    enum Pose_Components
    {
        Pose,
        RotMatrix,
        PointPos,
        PointAcc,
    };
    
    std::map<std::string, Pose_Components> _pose_option {
        { "Pose", Pose },
        { "RotMatrix", RotMatrix },
        { "PointPos", PointPos },
        { "PointAcc", PointAcc }
    };
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_Pose(XBot::ModelInterface::Ptr model,
                   std::string target_frame,
                   std::string source_frame,
                   std::string link_name,
                   std::vector<std::string> pose_list,
                   size_t in_start_port,
                   size_t out_start_port);
    
    ~XBotBlock_Pose(){};

    bool configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                               blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool setPoseOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

