#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>

#include <memory>
#include <string>

namespace XBB_MechanicalLimits {
    class XBotBlock_MechanicalLimits;
}

class XBB_MechanicalLimits::XBotBlock_MechanicalLimits
{
private:
    
    XBot::XBotInterface::Ptr _xbi;
    
    std::vector<std::string> _mechanical_limits_list;
    
    size_t _start_port;
    
    int _joint_number;

    enum Mechanical_Limits_Options
    {
        q_min,
        q_max,
        qdot_max,
        tau_max,
    };
    
    std::map<std::string, Mechanical_Limits_Options> _mechanical_limits_options {
        { "q_min", q_min },
        { "q_max", q_max },
        { "qdot_max", qdot_max },
        { "tau_max", tau_max }
    };

public:
 
    static const std::string ClassName;
    
    
    XBotBlock_MechanicalLimits(XBot::XBotInterface::Ptr xbi,
                               std::vector<std::string> mechanical_limits_list,
                               size_t start_port);
    ~XBotBlock_MechanicalLimits(){};

    void configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getMechanicalLimits(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
};

