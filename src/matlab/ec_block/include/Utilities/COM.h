#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>


namespace XBB_COM{
    class XBotBlock_COM;
}

class XBB_COM::XBotBlock_COM
{
private:

    XBot::ModelInterface::Ptr _model;
    std::string _reference_frame;
                                        
    uint8_t _model_id;
    int _n_joint;
    
    
    std::vector<std::string> _COM_list;
    
    size_t _start_port;

    enum COM_Components
    {
        p_COM,
        v_COM,
        a_COM,
        CM,
        CM_Matrix,
        CMMdotQdot,
    };
    
    std::map<std::string, COM_Components> _COM_option {
        { "p_COM", p_COM },
        { "v_COM", v_COM },
        { "a_COM", a_COM },
        { "CM", CM },
        { "CM_Matrix", CM_Matrix },
        { "CMMdotQdot", CMMdotQdot }
    };
    

public:
 
    static const std::string ClassName;
    
    XBotBlock_COM(XBot::ModelInterface::Ptr model,
                  std::string reference_frame,
                  std::vector<std::string> COM_list,
                  size_t start_port);
    
    ~XBotBlock_COM(){};

    bool configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool setCOMOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
    bool output(const blockfactory::core::BlockInformation* blockInfo);
};

