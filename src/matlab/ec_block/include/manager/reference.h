#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include "ec_block_utils.h"
#include <XBotInterface/Utils.h>

#include <memory>
#include <string>

namespace EcBlock {
    class Reference;
}

class EcBlock::Reference
{
public:
 
    enum class Mode
    {
        ROBOT,
        MODEL,
    };
    
private:
    
    XBot::XBotInterface::Ptr _xbi;
    
    Mode _mode;

    std::vector<std::string> _references_list;
    
    size_t _in_start_port,_out_start_port;
    
    bool _limits_check,_limits_enforce;
    
    int _joint_number;
    
    enum References_Options
    {
        qJ_ref,
        qM_ref,
        qJdot_ref,
        qMdot_ref,
        qJddot_ref,
        tau_ref,
        qTemp_ref,
        K_ref,
        D_ref,
    };
    
    std::map<std::string, References_Options> _references_options {
        { "qJ_ref", qJ_ref },
        { "qM_ref", qM_ref },
        { "qJdot_ref", qJdot_ref },
        { "qMdot_ref", qMdot_ref },
        { "qJddot_ref", qJddot_ref },
        { "tau_ref", tau_ref },
        { "qTemp_ref", qTemp_ref },
        { "K_ref", K_ref },
        { "D_ref", D_ref }
    };

public:
 
    static const std::string ClassName;
    
    
    Reference(XBot::XBotInterface::Ptr xbi,
                         Mode mode,
                         std::vector<std::string> references_list,
                         size_t in_start_port,
                         size_t out_start_port,
                         bool limits_check,
                         bool limits_enforce);
    ~Reference(){};

    void configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                               blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool setReferences(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

