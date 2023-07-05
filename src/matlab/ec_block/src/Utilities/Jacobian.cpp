#include "XBotBlock/Utilities/Jacobian.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM 7

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(XBotBlock_Jacobian, XBB_Jacobian::XBotBlock_Jacobian, blockfactory::core::Block);

using namespace XBB_Jacobian;


using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

unsigned XBotBlock_Jacobian::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool XBotBlock_Jacobian::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM] = {"ModelName","JacobianList","LinkName","TargetFrame","ExternalRef","TargetLink","BaseLink"}; // This label is used later to access the paramemeter
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

bool XBotBlock_Jacobian::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!XBotBlock_Jacobian::parseParameters(blockInfo)) {
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
    
    // get jacobian list parameter
    std::string jacobian_list;
    if (!m_parameters.getParameter("JacobianList", jacobian_list)) {
        bfError << "Failed to jacobian list";
        return false;
    }
    
    CommonUtils::checkParamSelected(jacobian_list,_jacobian_list);
    
    if(_jacobian_list.empty())
    {
       bfError << "NO param selected, please select at least one of it!"; 
       return false;
    }

    
    _link_name="";
    if (!m_parameters.getParameter("LinkName", _link_name)) {
        bfError << "Failed to parse link name parameter";
        return false;
    }
    
    _target_frame="";
    if (!m_parameters.getParameter("TargetFrame", _target_frame)) {
        bfError << "Failed to parse target frame parameter";
        return false;
    }
    
    std::string external_ref;
    if (!m_parameters.getParameter("ExternalRef", external_ref)) {
        bfError << "Failed to parse external reference parameter";
        return false;
    }
    
    // Check the content of the parameter
    if (external_ref == "Off") {
        _external_ref = ExtRef::Off;
    }
    else if (external_ref == "On") {
        _external_ref = ExtRef::On;
    }
    else {
        bfError << "External reference option " << external_ref << " not recognized";
        return false;
    }
    
     _target_link="";
    if (!m_parameters.getParameter("TargetLink", _target_link)) {
        bfError << "Failed to parse target link parameter";
        return false;
    }
    
    _base_link="";
    if (!m_parameters.getParameter("BaseLink", _base_link)) {
        bfError << "Failed to parse base link parameter";
        return false;
    }
      
    return true;
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_Jacobian::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
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
    blockfactory::core::OutputPortsInfo outputPortInfo;
    
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
    _is_floating_base=false;
    if(!model_retrieved)
    {
        // ignore errors during configureSizeAndPorts phase 
    }
    else
    {
        _n_joint=_model->getJointNum();
        _is_floating_base = _model->isFloatingBase();
    }

    bool jdotqdot_selected=false;
    
    for(size_t i=0; i < _jacobian_list.size();i++)
    {
        std::vector<int> port_size;
        if(_jacobian_option.count(_jacobian_list[i]) > 0)
        {
            switch(_jacobian_option.at(_jacobian_list[i]))
            {
                case J: {
                            port_size.push_back(6);
                            port_size.push_back(_n_joint);
                        }break;
                        
                case J_postural:    {
                                        if (_is_floating_base) 
                                        {
                                            port_size.push_back(_n_joint - 6);
                                        } 
                                        else 
                                        {
                                            port_size.push_back(_n_joint);
                                        }
                                        port_size.push_back(_n_joint);
                                    }break;
                            
                case J_relative:    {
                                        port_size.push_back(6);
                                        port_size.push_back(_n_joint);
                                    }break;
                                    
                case J_COM: {
                                    port_size.push_back(3);
                                    port_size.push_back(_n_joint);
                            }break;
                
                case dJcomQdot: {
                                    port_size.push_back(3);
                                }break;
                                    
                case JdotQdot:  {
                                    jdotqdot_selected=true;
                                    port_size.push_back(6);
                                }break;
                            
                case JdotQdot_relative: {
                                            port_size.push_back(6);
                                        }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized jacobian option from parameters selected";
            return false;
        }
        
        size_t index = outputPortInfo.size();
        
        blockfactory::core::Port::Info output{/*portIndex=*/index,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }
    
    if(_external_ref==ExtRef::On)
    {
        blockfactory::core::Port::Info external_reference{/*portIndex=*/inputPortInfo.size(),
                                                            std::vector<int>{3},
                                                            blockfactory::core::Port::DataType::DOUBLE};

        inputPortInfo.push_back(external_reference);  
    }
    
    if(jdotqdot_selected)
    {
        blockfactory::core::Port::Info point{/*portIndex=*/inputPortInfo.size(),
                                             std::vector<int>{3},
                                             blockfactory::core::Port::DataType::DOUBLE};

        inputPortInfo.push_back(point);  
    }

    // Store the port information into the BlockInformation
   if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
   }
        
    return true;
}

bool XBotBlock_Jacobian::setJacobianOut(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
        for(size_t i=0; i < _jacobian_list.size();i++)
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i );
            // Check the signal validity
            if (!output) {
                bfError << "Signal not valid";
                return false;
            }
            // verify if the element of the list exists like option
            if(_jacobian_option.count(_jacobian_list[i]) > 0)
            {
                Eigen::MatrixXd J_out;
                switch(_jacobian_option.at(_jacobian_list[i]))
                {
                    case J: {
                                if(_external_ref==ExtRef::Off)
                                {
                                    if(_target_frame!="")
                                    {
                                        if(!_model->getJacobian(_link_name,_target_frame,J_out))
                                        {
                                            bfError << "invalid link name or target frame";
                                            return false;
                                        }
                                    }
                                    else
                                    {
                                        if(!_model->getJacobian(_link_name,J_out))
                                        {
                                            bfError << "invalid link name";
                                            return false;
                                        }
                                    }
                                }
                                else
                                {
                                    // Get general signal input
                                    blockfactory::core::InputSignalPtr ext_ref_signal = blockInfo->getInputPortSignal(/*index=*/1);
                                    
                                    // Check the signal validity
                                    if (!ext_ref_signal) {
                                        bfError << "external point reference input signal not valid";
                                        return false;
                                    }
                                    
                                    for (size_t  k=0; k < _rp.size(); ++k)
                                    {
                                        _rp[k]=ext_ref_signal->get<double>(k);
                                    }

                                    if(!_model->getJacobian(_link_name,_rp,J_out))
                                    {
                                        bfError << "invalid link name";
                                        return false;
                                    }  
                                }     
                            }break;
                                
                     case J_postural:   {
                                            _model->getPosturalJacobian(J_out);
                                        }break;
                    case J_relative:    {
                                            if(!_model->getRelativeJacobian(_target_link,_base_link,J_out))
                                            {
                                                bfError << "invalid target or base link";
                                                return false;
                                            }
                                        }break;
                    
                    case J_COM: {
                                    _model->getCOMJacobian(J_out);
                                }break;
                    
                    case dJcomQdot: {
                                        Eigen::Vector3d dJcomQdot;
                                        _model->getCOMJacobian(J_out,dJcomQdot);
                                        for (size_t  k=0; k < dJcomQdot.size(); ++k)
                                        {
                                            output->set(k, dJcomQdot[k]);
                                        }
                                        
                                    }break;
                                        
                    case JdotQdot:  {
                                        size_t port_n=1; // port zero= model_id
                                        if(_external_ref==ExtRef::On)
                                        {   
                                            port_n = port_n +1;
                                        }
                                         // Get general signal input
                                        blockfactory::core::InputSignalPtr point_signal = blockInfo->getInputPortSignal(/*index=*/port_n);
                                        
                                        // Check the signal validity
                                        if (!point_signal) {
                                            bfError << "external point input signal not valid";
                                            return false;
                                        }
                                        
                                        for (size_t  k=0; k < _point.size(); ++k)
                                        {
                                            _point[k]=point_signal->get<double>(k);
                                        }
                                        
                                        Eigen::Vector6d jdotqdot;
                                        
                                        if(!_model->computeJdotQdot(_link_name,_point,jdotqdot))
                                        {
                                            bfError << "invalid link name";
                                            return false;
                                        }
                                        
                                        for (size_t  k=0; k < jdotqdot.size(); ++k)
                                        {
                                            output->set(k, jdotqdot[k]);
                                        }
                                        
                                    }break;
                    case JdotQdot_relative: {
                                                Eigen::Vector6d jdotqdot;
                                                
                                                if(!_model->computeRelativeJdotQdot(_target_link,_base_link,jdotqdot))
                                                {
                                                     bfError << "invalid target or base link";
                                                    return false;
                                                }
                                                
                                                for (size_t  k=0; k < jdotqdot.size(); ++k)
                                                {
                                                    output->set(k, jdotqdot[k]);
                                                }
                                            }break;
                    
                }
                
                if(_jacobian_list[i] != "JdotQdot" && 
                   _jacobian_list[i] !="JdotQdot_relative" &&
                   _jacobian_list[i] != "dJcomQdot")
                {
                    int rows;
                    int cols =_n_joint;
                    if(_jacobian_list[i] == "J_postural")
                    {
                        if (_is_floating_base) 
                        {
                            rows = _n_joint -6;
                        }
                        else
                        {
                            rows = _n_joint;
                        }
                    }
                    else if(_jacobian_list[i] == "J_COM")
                    {
                        rows=3;
                    }
                    else
                    {
                        rows=6;
                    }
                    // Allocate objects for row-major -> col-major conversion
                    Eigen::Map<MatrixXdSimulink> JacobianColMajor(output->getBuffer<double>(),rows,cols);
                    JacobianColMajor=J_out;
                }
            }
            else
            {
                bfError << "Found unrecognized pose components option from parameters selected";
                return false;
            }
        }
    }
    else
    {
        bfError << "Try to perform an operation on a null model";
        return false;
    }

    return true;
}


bool XBotBlock_Jacobian::initialize(blockfactory::core::BlockInformation* blockInfo)
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
    _is_floating_base=false;
    
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
        _is_floating_base = _model->isFloatingBase();
    }
    
    if(!setJacobianOut(blockInfo))
    {
        return false;
    }
    return true;
}

bool XBotBlock_Jacobian::output(const blockfactory::core::BlockInformation* blockInfo)
{
    
    if(!setJacobianOut(blockInfo))
    {
        return false;
    }
    return true;
    
}

bool XBotBlock_Jacobian::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    CommonUtils::clearModelMap();
    return true;
    
}
