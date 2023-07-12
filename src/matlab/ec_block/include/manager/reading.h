#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include "ec_block_utils.h"

namespace EcBlock {
    class Reading;
}

class EcBlock::Reading
{
private:
    
    EcIface::Ptr _robot;
    std::vector<int> _q_id;
    int _ctrl_mode;

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
        tau,
        qTemp,
        bTemp,
        gainP,
        gainD,
        qJ_ref,
        qJdot_ref,
        tau_ref,
        fault,
        cmd_aux_sts,
    };
    
    std::map<std::string, Readings_Options> _readings_options {
        { "q_ID", q_ID },
        { "qJ", qJ },
        { "qM", qM },
        { "qJdot", qJdot },
        { "qMdot", qMdot },
        { "tau", tau },
        { "qTemp", qTemp },
        { "bTemp", bTemp },
        { "gainP", gainP },
        { "gainD", gainD },
        { "qJ_ref", qJ_ref },
        { "qJdot_ref", qJdot_ref },
        { "tau_ref", tau_ref },
        { "fault", fault },
        { "cmd_aux_sts",cmd_aux_sts }
    };

public:
 
    static const std::string ClassName;
    
    
    Reading(EcIface::Ptr robot,
            std::vector<std::string> readings_list,
            size_t start_port);
    ~Reading(){};

    void configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getReadings(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map);
    bool output(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map);
};

