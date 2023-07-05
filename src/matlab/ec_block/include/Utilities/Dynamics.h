#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>


namespace XBB_Dynamics{
    class XBotBlock_Dynamics;
}

class XBB_Dynamics::XBotBlock_Dynamics : public blockfactory::core::Block
{
private:

    XBot::ModelInterface::Ptr _model;
    int _n_joint;
    Eigen::MatrixXd _B,_BInv;
    Eigen::VectorXd _H,_g,_tau_id,_tau_id_const,_weight;
    Eigen::MatrixXd _J;
    double _mass;
    std::string _model_name;
    uint8_t _model_id;
    std::vector<std::string> _dynamics_components_list;
    

    enum Dynamics_Components
    {
        B,
        BInv,
        M,
        H,
        tau_g,
        tau_id,
        tau_id_const,
    };
    
    std::map<std::string, Dynamics_Components> _dynamics_components_option {
        { "B", B },
        { "BInv", BInv },
        { "M", M },
        { "H", H },
        { "tau_g", tau_g },
        { "tau_id", tau_id },
        { "tau_id_const", tau_id_const }
    };

   
public:
 
    static const std::string ClassName;
    
    XBotBlock_Dynamics() = default;
    ~XBotBlock_Dynamics() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool readParameters(blockfactory::core::BlockInformation* blockInfo);
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool set_DynamicsOut(const blockfactory::core::BlockInformation* blockInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
};

