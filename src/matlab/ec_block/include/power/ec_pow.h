#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <common/ec_block_utils.h>

namespace EcBlock {
    class Pow;
}

class EcBlock::Pow
{
private:

    std::vector<int> _pow_id;
    std::vector<std::string> _pow_list;
    
    PwrStatusMap _pow_status_map;
    
    size_t _start_port;
    int _pow_number;

    enum pow_component
    {
        pow_id,
        v_batt,
        v_load,
        i_load,
        temp_pcb,
        temp_heatsink,
        temp_batt,
    };
    
    
    std::map<std::string, pow_component> _pow_option {
        { "pow_id", pow_id },
        { "v_batt", v_batt },
        { "v_load", v_load },
        { "i_load", i_load },
        { "temp_pcb", temp_pcb },
        { "temp_heatsink", temp_heatsink },
        { "temp_batt", temp_batt }
    };
    

public:
 
    static const std::string ClassName;

    Pow(std::vector<std::string> pow_list,size_t start_port);
    ~Pow(){};

    bool configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getPow(const blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map);
    bool output(const blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map);
};

