#include "XBotBlock/Utilities/Kinematics.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>


#define PARAM 9

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(XBotBlock_Kinematics, XBB_Kinematics::XBotBlock_Kinematics, blockfactory::core::Block);


using Matrix4dSimulink = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
using Matrix3dSimulink = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;

using namespace XBB_Kinematics;

unsigned XBotBlock_Kinematics::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool XBotBlock_Kinematics::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM] = {"ModelName",
                               "PoseList",
                               "SourceFrame",
                               "TargetFrame",
                               "TwistList",
                               "LinkName",
                               "BaseLinkName",
                               "COMList",
                               "ReferenceFrame"}; // This label is used later to access the paramemeter
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
bool XBotBlock_Kinematics::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!XBotBlock_Kinematics::parseParameters(blockInfo)) {
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
    
    // get pose list parameter
    std::string pose_list;
    if (!m_parameters.getParameter("PoseList", pose_list)) {
        bfError << "Failed to parse pose list";
        return false;
    }
    
    CommonUtils::checkParamSelected(pose_list,_pose_list);
    
    _source_frame="";

    if (!m_parameters.getParameter("SourceFrame", _source_frame)) {
        bfError << "Failed to parse source frame parameter";
        return false;
    }
    
    _target_frame="";
  
    if (!m_parameters.getParameter("TargetFrame", _target_frame)) {
        bfError << "Failed to parse target frame parameter";
        return false;
    }
    
    
    // get twist list parameter
    std::string twist_list;
    if (!m_parameters.getParameter("TwistList", twist_list)) {
        bfError << "Failed to parse twist list";
        return false;
    }
    
    CommonUtils::checkParamSelected(twist_list,_twist_list);

    _link_name="";

    if (!m_parameters.getParameter("LinkName", _link_name)) {
        bfError << "Failed to parse link name parameter";
        return false;
    }
    
    _base_link_name="";
  
    if (!m_parameters.getParameter("BaseLinkName", _base_link_name)) {
        bfError << "Failed to parse base link name parameter";
        return false;
    }
    
    // get COM list parameter
    std::string COM_list;
    if (!m_parameters.getParameter("COMList", COM_list)) {
        bfError << "Failed to parse COM list";
        return false;
    }
    
    CommonUtils::checkParamSelected(COM_list,_COM_list);

    
    _reference_frame="";
  
    if (!m_parameters.getParameter("ReferenceFrame", _reference_frame)) {
        bfError << "Failed to parse Reference Frame parameter";
        return false;
    }
    
    if( _pose_list.empty() && _twist_list.empty() &&_COM_list.empty())
    {
       bfError << "NO param selected, please select at least one of it!"; 
       return false;
    }
    
    return true;
    
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_Kinematics::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
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
    
    if(!model_retrieved)
    {
        // ignore errors during configureSizeAndPorts phase 
    }

    // create pose class configuring size and ports.
    if(!_pose_list.empty())
    {
        _pose_ptr = std::make_shared<XBB_Pose::XBotBlock_Pose>(_model,
                                                               _target_frame,
                                                               _source_frame,
                                                               _link_name,
                                                               _pose_list,
                                                               inputPortInfo.size(),
                                                               outputPortInfo.size());
        if(!_pose_ptr->configureSizeAndPorts(inputPortInfo,outputPortInfo))
        {
            return false;
        }
    }
    

    // create twist class configuring size and ports.
    if(!_twist_list.empty())
    {
        _twist_ptr = std::make_shared<XBB_Twist::XBotBlock_Twist>(_model,
                                                                  _link_name,
                                                                  _base_link_name,
                                                                  _twist_list,
                                                                  outputPortInfo.size());
        _twist_ptr->configureSizeAndPorts(outputPortInfo);

    }
    
    // create COM class configuring size and ports.
    if(!_COM_list.empty())
    {
        _COM_ptr = std::make_shared<XBB_COM::XBotBlock_COM>(_model,
                                                            _reference_frame,
                                                            _COM_list,
                                                            outputPortInfo.size());
        if(!_COM_ptr->configureSizeAndPorts(outputPortInfo))
        {
            return false;
        }
    }
    

    // Store the port information into the BlockInformation
   if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
   }
        
    return true;
}


bool XBotBlock_Kinematics::initialize(blockfactory::core::BlockInformation* blockInfo)
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
    
    size_t in_start_port=0;
    size_t out_start_port=0;
    
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
        else
        {
            in_start_port = in_start_port +1 ;
            // create pose class and initialization.
            if(!_pose_list.empty())
            {
                _pose_ptr = std::make_shared<XBB_Pose::XBotBlock_Pose>(_model,
                                                                       _target_frame,
                                                                       _source_frame,
                                                                       _link_name,
                                                                       _pose_list,
                                                                       in_start_port,
                                                                       out_start_port);
                if(!_pose_ptr->initialize(blockInfo))
                {
                    return false;
                }
            }
            
            out_start_port = out_start_port + _pose_list.size();

            // create twist class and initialization.
            if(!_twist_list.empty())
            {
                _twist_ptr = std::make_shared<XBB_Twist::XBotBlock_Twist>(_model,
                                                                        _link_name,
                                                                        _base_link_name,
                                                                        _twist_list,
                                                                        out_start_port);
                if(!_twist_ptr->initialize(blockInfo))
                {
                    return false;
                }
            }
            
            out_start_port = out_start_port + _twist_list.size();
            
            // create COM class and initialization.
            if(!_COM_list.empty())
            {
                _COM_ptr = std::make_shared<XBB_COM::XBotBlock_COM>(_model,
                                                                    _reference_frame,
                                                                    _COM_list,
                                                                    out_start_port);
                if(!_COM_ptr->initialize(blockInfo))
                {
                    return false;
                }
            }
        }
    }
    
    return true;
}


bool XBotBlock_Kinematics::output(const blockfactory::core::BlockInformation* blockInfo)
{
    // get pose 
    if(_pose_ptr != nullptr)
    {
        if(!_pose_ptr->output(blockInfo))
        {
            return false;
        }
    }
    // get twist 
    if(_twist_ptr != nullptr)
    {
        if(!_twist_ptr->output(blockInfo))
        {
            return false;
        }
    }
    // get COM 
    if(_COM_ptr != nullptr)
    {
        if(!_COM_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    return true;
}

bool XBotBlock_Kinematics::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    CommonUtils::clearModelMap();
    return true;
    
}
