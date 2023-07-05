#include "XBotBlock/Utilities/Dynamics.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM 2

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(XBotBlock_Dynamics, XBB_Dynamics::XBotBlock_Dynamics, blockfactory::core::Block);

using namespace XBB_Dynamics;
using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

unsigned XBotBlock_Dynamics::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool XBotBlock_Dynamics::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM] = {"ModelName","ComponentsList"}; // This label is used later to access the paramemeter
    auto type = blockfactory::core::ParameterType::STRING;
   
    for(unsigned i=0;i<PARAM;i++)
    {
        // Create the parameter
        blockfactory::core::ParameterMetadata parameterMetadata(type, index+i, rows, cols, name[i]);

        // Add the parameter metadata into the BlockInformation
        if (!blockInfo->addParameterMetadata(parameterMetadata)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }
   
    // Ask to the BlockInformation interface to parse the parameters and store them into
    // the protected m_parameters member of the parent blockfactory::core::Block class.
    bool paramParsedOk = blockInfo->parseParameters(m_parameters);
   
    // Return the outcome of the parameter parsing.
    // If the parsing fails, the execution stops.
    return paramParsedOk;
}
bool XBotBlock_Dynamics::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!XBotBlock_Dynamics::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }
    
   // get model name parameter
    _model_name="";
    if (!m_parameters.getParameter("ModelName", _model_name)) {
        bfError << "Failed to parse model name parameter";
        return false;
    }   
    
    if(_model_name=="")
    {
        bfError << "Missing model name";
        return false;
    }
    
    // get components list parameter
    std::string components_list;
    if (!m_parameters.getParameter("ComponentsList", components_list)) {
        bfError << "Failed to parse dynamics components list";
        return false;
    }
    
    CommonUtils::checkParamSelected(components_list,_dynamics_components_list);
    
    if(_dynamics_components_list.empty())
    {
       bfError << "NO param selected, please select at least one of it!"; 
       return false;
    }
    
    return true;
    
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_Dynamics::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
{
   // The base blockfactory::core::Block class needs to be configured first
    if (!blockfactory::core::Block::configureSizeAndPorts(blockInfo)) {
       return false;
    }

    // read paramemeters of the block
    if (!readParameters(blockInfo)) {
       return false;
    }
    
    
    // Store together the port information objects
    blockfactory::core::InputPortsInfo inputPortInfo;
    
    blockfactory::core::Port::Info model_id_in_port{/*portIndex=*/0,
                                                    std::vector<int>{1},
                                                    blockfactory::core::Port::DataType::DOUBLE};

    inputPortInfo.push_back(model_id_in_port);  
    
    // retrieve model and model id
    std::string error_info = "";
    bool allow_new_model=false;
    _model_id=0;
    bool model_retrieved = CommonUtils::RetrieveModel("",_model_name,allow_new_model,_model_id,_model,error_info);
    
    _n_joint = 1;
    if(!model_retrieved)
    {
        // ignore errors during configureSizeAndPorts phase 
    }
    else
    {
        _n_joint=_model->getJointNum();
    }

    // Store together the port information objects
    blockfactory::core::OutputPortsInfo outputPortInfo;
    
    for(size_t i=0; i < _dynamics_components_list.size();i++)
    {
        std::vector<int> port_size;
        if(_dynamics_components_option.count(_dynamics_components_list[i]) > 0)
        {
            switch(_dynamics_components_option.at(_dynamics_components_list[i]))
            {
                case B: {
                            port_size.push_back(_n_joint);
                            port_size.push_back(_n_joint);
                        }break;
                        
                case BInv:  {
                                port_size.push_back(_n_joint);
                                port_size.push_back(_n_joint);
                            }break;
                case M: {
                            port_size.push_back(1);
                        }break;
                case H: {
                            port_size.push_back(_n_joint);
                        }break;
                case tau_g: {
                                port_size.push_back(_n_joint);
                            }break;  
                case tau_id:    {
                                    port_size.push_back(_n_joint);
                                }break;
                case tau_id_const:  {
                                        port_size.push_back(_n_joint);
                                        blockfactory::core::Port::Info jacobian{/*portIndex=*/1,
                                                                                std::vector<int>{6,_n_joint},
                                                                                blockfactory::core::Port::DataType::DOUBLE};
                                        blockfactory::core::Port::Info weight{/*portIndex=*/2,
                                                                                std::vector<int>{_n_joint},
                                                                                blockfactory::core::Port::DataType::DOUBLE};

                                        inputPortInfo.push_back(jacobian);  
                                        inputPortInfo.push_back(weight);   
                                        
                                    }break; 
            }
            
        }
        else
        {
            bfError << "Found unrecognized dynamics components option from parameters selected";
            return false;
        }
        
        blockfactory::core::Port::Info output{/*portIndex=*/i,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }                                                           
    // Store the port information into the BlockInformation
   if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
   }
        
    return true;
}

bool XBotBlock_Dynamics::set_DynamicsOut(const blockfactory::core::BlockInformation* blockInfo)
{
    for(size_t i=0; i < _dynamics_components_list.size();i++)
    {
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i);
        // Check the signal validity
        if (!output) {
            bfError << "Signal not valid";
            return false;
        }
        
        if(_dynamics_components_option.count(_dynamics_components_list[i]) > 0)
        {
            switch(_dynamics_components_option.at(_dynamics_components_list[i]))
            {
                case B: {
                            _model->getInertiaMatrix(_B);
                            // Allocate objects for row-major -> col-major conversion
                            Eigen::Map<MatrixXdSimulink> massInertiaColMajor(output->getBuffer<double>(),_n_joint,_n_joint);
                            massInertiaColMajor = _B;
                        }break;
                        
                case BInv:  {
                                _model->getInertiaInverse(_BInv);
                                // Allocate objects for row-major -> col-major conversion
                                Eigen::Map<MatrixXdSimulink> massInertiaInvColMajor(output->getBuffer<double>(),_n_joint,_n_joint);
                                massInertiaInvColMajor= _BInv;
                            }break;
                            
                case M: {
                            _mass=_model->getMass(); 
                            output->set(0, _mass);
                        }break;
                            
                case H: {
                            _model->computeNonlinearTerm(_H); 
                            for (size_t  k=0; k < output->getWidth(); ++k)
                            {
                                output->set(k, _H[k]);
                            }
                        }break;
                case tau_g: {
                                _model->computeGravityCompensation(_g); 
                                for (size_t  k=0; k < output->getWidth(); ++k)
                                {
                                    output->set(k, _g[k]);
                                }
                            }break; 
                case tau_id:    {
                                    _model->computeInverseDynamics(_tau_id); 
                                    for (size_t  k=0; k < output->getWidth(); ++k)
                                    {
                                        output->set(k, _tau_id[k]);
                                    }
                                }break; 
                case tau_id_const:  {
                                        // Get general signal input
                                        blockfactory::core::InputSignalPtr jacobian_signal = blockInfo->getInputPortSignal(/*index=*/1);
                                        
                                        // Check the signal validity
                                        if (!jacobian_signal) {
                                            bfError << "Jacobian signal not valid";
                                            return false;
                                        }
                                        const double* jacobian_buffer = jacobian_signal->getBuffer<double>();
                                        if (!jacobian_buffer) {
                                            bfError << "Failed to read jacobian input signal.";
                                            return false;
                                        }

                    
                                        _J.matrix() = Eigen::Map<MatrixXdSimulink>(const_cast<double*>(jacobian_signal->getBuffer<double>()),6,_n_joint);

                                        blockfactory::core::InputSignalPtr weight_signal = blockInfo->getInputPortSignal(/*index=*/2);
                                        
                                        if (!weight_signal) {
                                            bfError << "Weight signal not valid";
                                            return false;
                                        }
                                    
                                        if(_weight.size() != weight_signal->getWidth())
                                        {
                                            bfError << "Different dimension of weight port: " + std::to_string(output->getWidth()) +" and joint number: " + std::to_string(_weight.size());
                                            return false;
                                        }
                                        
                                        // set the signal output
                                        for (size_t  k=0; k < weight_signal->getWidth(); ++k)
                                        {
                                            _weight[k]=weight_signal->get<double>(k);
                                        }    
                                        
                                        if(!_model->computeConstrainedInverseDynamics(_J,_weight,_tau_id_const))
                                        {
                                            bfError << "Invalid Jacobian or weight dimension.";
                                            return false;  
                                        }
                                        
                                        for (size_t  k=0; k < output->getWidth(); ++k)
                                        {
                                            output->set(k, _tau_id_const[k]);
                                        }
                                                
                                        
                                    }break; 
            }
            
        }
        else
        {
            bfError << "Found unrecognized dynamics components option from parameters selected";
            return false;
        }
    }

   return true;
}

bool XBotBlock_Dynamics::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    // The base blockfactory::core::Block class need to be initialized first
    if (!Block::initialize(blockInfo)) {
        return false;
    }
   
    // read paramemeters of the block
    if (!readParameters(blockInfo)) {
       return false;
    }
    
     // Get the model id signal
    blockfactory::core::InputSignalPtr model_id_signal = blockInfo->getInputPortSignal(/*index=*/0);
    
    // Check the signal validity
    if (!model_id_signal) {
        bfError << "Model id signal not valid";
        return false;
    }
    
    uint8_t model_id_read = (uint8_t) model_id_signal->get<double>(0);
        
    // check model id read validity
    if(!CommonUtils::isValidModelID(model_id_read))
    {
        bfError << "model id read: " << std::to_string(model_id_read) << " is not a valid model id";
        return false;
    }
    
    // retrieve model and model id
    std::string error_info = "";
    bool allow_new_model=false;
    _model_id=0;
    bool model_retrieved = CommonUtils::RetrieveModel("",_model_name,allow_new_model,_model_id,_model,error_info);
    
    _n_joint = 1;
    
    if(!model_retrieved)
    {
        bfError << "Model not retrieved, reason: " << error_info;
        return false;
    }
    else
    {
        //consistency check between model id retrieved and model id read
        if(_model_id != model_id_read)
        {
            bfError << "Model ID mismatch between the model name of the block and with the physical signal" << error_info;
            return false;
        }
        _n_joint=_model->getJointNum();
    }
    
    _weight.resize(_n_joint);
    
    if(!set_DynamicsOut(blockInfo))
    {
        return false;
    }
    return true;
}

bool XBotBlock_Dynamics::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(!set_DynamicsOut(blockInfo))
    {
        return false;
    }
    return true;
}

bool XBotBlock_Dynamics::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    CommonUtils::clearModelMap();
    return true;
    
}
