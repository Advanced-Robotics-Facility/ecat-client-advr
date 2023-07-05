#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>

namespace XBB_Jacobian{
    class XBotBlock_Jacobian;
}

class XBB_Jacobian::XBotBlock_Jacobian : public blockfactory::core::Block
{
private:
    XBot::ModelInterface::Ptr _model;
    std::string _model_name,_link_name,_target_frame,_target_link,_base_link;
    int _n_joint;
    bool _is_floating_base;
    Eigen::Vector3d _rp,_point;
    uint8_t _model_id;
    
    std::vector<std::string> _jacobian_list;

    enum Jacobian_Option
    {
        J,
        J_postural,
        J_relative,
        J_COM,
        dJcomQdot,
        JdotQdot,
        JdotQdot_relative,
    };
    
    std::map<std::string, Jacobian_Option> _jacobian_option {
        { "J", J },
        { "J_postural", J_postural },
        { "J_relative", J_relative },
        { "J_COM", J_COM },
        { "dJcomQdot", dJcomQdot },
        { "JdotQdot", JdotQdot },
        { "JdotQdot_relative", JdotQdot_relative }
    };
    
    enum class ExtRef
    {
        Off,
        On,
    };

    ExtRef _external_ref;
    
public:
 
    static const std::string ClassName;
    
    
    XBotBlock_Jacobian() = default;
    ~XBotBlock_Jacobian() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool setJacobianOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

