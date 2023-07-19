#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <common/ec_block_utils.h>

namespace EcBlock {
    class Reading;
}

class EcBlock::Reading
{
private:
    std::vector<int> _q_id;
    std::vector<double> _q_home,_q_trj;
    
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
        q_home,
        q_trj,
        qJ_ref,
        qJdot_ref,
        tau_ref,
        gainP_ref,
        gainD_ref,
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
        { "q_home", q_home },
        { "q_trj", q_trj },
        { "qJ_ref", qJ_ref },
        { "qJdot_ref", qJdot_ref },
        { "tau_ref", tau_ref },
        { "gainP_ref", gainP_ref },
        { "gainD_ref", gainD_ref },
        { "fault", fault },
        { "cmd_aux_sts",cmd_aux_sts }
    };

public:
 
    static const std::string ClassName;
    
    
    Reading(std::vector<std::string> readings_list,
            size_t start_port);
    ~Reading(){};

    void configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getReadings(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref);
    bool output(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref);
};

