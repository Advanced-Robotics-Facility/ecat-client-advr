#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>


namespace XBB_Twist{
    class XBotBlock_Twist;
}

class XBB_Twist::XBotBlock_Twist
{
private:

    XBot::ModelInterface::Ptr _model;
    std::string _link_name,_base_link_name;
    
    std::vector<std::string> _twist_list;
    
    size_t _start_port;

    enum Twist_Components
    {
        v_tw,
        a_tw,
        v_tw_rel,
        a_tw_rel,
    };
    
    std::map<std::string, Twist_Components> _twist_option {
        { "v_tw", v_tw },
        { "a_tw", a_tw },
        { "v_tw_rel", v_tw_rel },
        { "a_tw_rel", a_tw_rel }
    };
    

public:
 
    static const std::string ClassName;

    XBotBlock_Twist(XBot::ModelInterface::Ptr model,
                    std::string link_name,
                    std::string base_link_name,
                    std::vector<std::string> twist_list,
                    size_t start_port);
    ~XBotBlock_Twist(){};

    void configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool getTwist(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

