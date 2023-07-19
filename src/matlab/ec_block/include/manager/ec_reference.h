#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <common/ec_block_utils.h>

namespace EcBlock {
    class Reference;
}

class EcBlock::Reference
{
public:
 
private:

    std::vector<std::string> _references_list;
    
    size_t _in_start_port,_out_start_port;
    
    int _joint_number;
    std::vector<int> _q_id;
    int _ctrl_mode;
    
    enum References_Options
    {
        qJ_ref,
        qJdot_ref,
        tau_ref,
        gainP_ref,
        gainD_ref,
    };
    
    std::map<std::string, References_Options> _references_options {
        { "qJ_ref", qJ_ref },
        { "qJdot_ref", qJdot_ref },
        { "tau_ref", tau_ref },
        { "gainP_ref", gainP_ref },
        { "gainD_ref", gainD_ref }
    };

public:
 
    static const std::string ClassName;
    
    
    Reference(std::vector<std::string> references_list,
              size_t in_start_port);
    ~Reference(){};

    void configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo);
    bool setReferences(const blockfactory::core::BlockInformation* blockInfo,std::vector<MR> &motors_ref,std::string &error_info);
    bool output(const blockfactory::core::BlockInformation* blockInfo,std::vector<MR> &motors_ref);
};

