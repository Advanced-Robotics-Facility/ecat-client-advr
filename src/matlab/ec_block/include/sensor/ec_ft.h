#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <common/ec_block_utils.h>

namespace EcBlock {
    class Ft;
}

class EcBlock::Ft
{
private:

    std::vector<int> _ft_id;
    std::vector<std::string> _ft_list;
    
    FtStatusMap _ft_status_map;
    
    size_t _start_port;
    int _ft_number;

    enum ft_component
    {
        ft_id,
        F,
        T,
    };
    
    std::map<std::string, ft_component> _ft_option {
        { "ft_id", ft_id },
        { "F", F },
        { "T", T }
    };
    

public:
 
    static const std::string ClassName;

    Ft(std::vector<std::string> ft_list,size_t start_port);
    ~Ft(){};

    bool configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getFt(const blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map);
    bool output(const blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map);
};

