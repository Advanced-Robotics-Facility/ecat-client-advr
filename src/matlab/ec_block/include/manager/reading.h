#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>

#include <memory>
#include <string>

namespace EcBlock {
    class Reading;
}

class EcBlock::Reading
{
private:
    
    XBot::XBotInterface::Ptr _xbi;

    std::vector<std::string> _readings_list;

    size_t _start_port;
    
    int _joint_number;

    enum Readings_Options
    {
        q_ID,
        qJ,
        qM,
        qJdot,
        qMdot,
        qJddot,
        tau,
        qTemp,
        K,
        D,
        qJ_ref,
        qJdot_ref,
        tau_ref,
    };
    
    std::map<std::string, Readings_Options> _readings_options {
        { "q_ID", q_ID },
        { "qJ", qJ },
        { "qM", qM },
        { "qJdot", qJdot },
        { "qMdot", qMdot },
        { "qJddot", qJddot },
        { "tau", tau },
        { "qTemp", qTemp },
        { "K", K },
        { "D", D },
        { "qJ_ref", qJ_ref },
        { "qJdot_ref", qJdot_ref },
        { "tau_ref", tau_ref }
    };

public:
 
    static const std::string ClassName;
    
    
    Reading(XBot::XBotInterface::Ptr xbi,
                       std::vector<std::string> readings_list,
                       size_t start_port);
    ~Reading(){};

    void configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getReadings(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

