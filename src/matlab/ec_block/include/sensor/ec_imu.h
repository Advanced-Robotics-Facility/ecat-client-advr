#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <common/ec_block_utils.h>

namespace EcBlock {
    class Imu;
}

class EcBlock::Imu
{
private:

    std::vector<int> _imu_id;
    std::vector<std::string> _imu_list;
    
    ImuStatusMap _imu_status_map;
    
    size_t _start_port;
    int _imu_number;

    enum imu_component
    {
        imu_id,
        quat,
        a,
        omega,
    };
    
    std::map<std::string, imu_component> _imu_option {
        { "imu_id", imu_id },
        { "quat", quat },
        { "a", a },
        { "omega", omega }
    };
    

public:
 
    static const std::string ClassName;

    Imu(std::vector<std::string> imu_list,size_t start_port);
    ~Imu(){};

    bool configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getImu(const blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map);
    bool output(const blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map);
};

