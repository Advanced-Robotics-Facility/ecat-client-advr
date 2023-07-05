#include "XBotBlock/Managers/ModelManager.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM_NUM 14

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(XBotBlock_ModelManager, XBB_ModelManager::XBotBlock_ModelManager, blockfactory::core::Block);

using Matrix4dSimulink = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;

using namespace XBB_ModelManager;

unsigned XBotBlock_ModelManager::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM_NUM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool XBotBlock_ModelManager::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM_NUM] = {"ModelName",
                                   "RobotName",
                                   "UseExtModel",
                                   "SyncWithRobot",
                                   "GetFloatingBaseList",
                                   "SetFloatingBaseComponent",
                                   "GravityRequest",
                                   "ReferenceFrame",
                                   "ReadingsList",
                                   "MechanicalLimitsList",
                                   "MechanicalLimitsCheck",
                                   "MechanicalLimitsEnforce",
                                   "SensorList",
                                   "ReferencesList"}; // This label is used later to access the paramemeter
                                   
    auto type = blockfactory::core::ParameterType::STRING;
   
    for(unsigned i=0;i<PARAM_NUM;i++)
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

bool XBotBlock_ModelManager::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!XBotBlock_ModelManager::parseParameters(blockInfo)) {
        bfError <<  "Failed to parse parameters.";
        return false;
    }
    
    // get model name parameter
    _model_name="";
    if (!m_parameters.getParameter("ModelName", _model_name)) {
        bfError << "Failed to parse mnodel name parameter";
        return false;
    }   
    
    // check model name is an empty string
    if(_model_name=="")
    {
        bfError << "Missing model name";
        return false;
    }
    
    // get robot name parameter
    _robot_name="";
    if (!m_parameters.getParameter("RobotName", _robot_name)) {
        bfError << "Failed to parse robot name parameter";
        return false;
    }   
    
    // check robot name is an empty string
    if(_robot_name=="")
    {
        bfError << "Missing Robot Name";
        return false;
    }
    
    // get use external model parameter
    std::string use_ext_model="";
    if (!m_parameters.getParameter("UseExtModel", use_ext_model)) {
        bfError << "Failed to parse use external model paramemeter";
        return false;
    }
    
    // Check the content of the parameter
    if (use_ext_model == "Off") {
        _use_ext_model = UseExtModel::OFF;
    }
    else if (use_ext_model == "On") {
        _use_ext_model = UseExtModel::ON;
    }
    else {
        bfError << "Use external option:  " << use_ext_model << " not recognized";
        return false;
    }
    
    // get synchronized with robot parameter
    std::string sync_with_robot="";
    if (!m_parameters.getParameter("SyncWithRobot", sync_with_robot)) {
        bfError << "Failed to parse sync with model paramemeter";
        return false;
    }
    
    // Check the content of the parameter
    if (sync_with_robot == "Off") {
        _sync_with_robot = SyncWithRobot::OFF;
    }
    else if (sync_with_robot == "On") {
        _sync_with_robot = SyncWithRobot::ON;
    }
    else {
        bfError << "Sync model option:  " << sync_with_robot << " not recognized";
        return false;
    }
    
    // get get flaoting base list parameter
    std::string get_floating_base_list;
    if (!m_parameters.getParameter("GetFloatingBaseList", get_floating_base_list)) {
        bfError << "Failed to get flaoting base list";
        return false;
    }
    
    CommonUtils::checkParamSelected(get_floating_base_list,_get_floating_base_list);
    
    _set_floating_base_component="";

    if (!m_parameters.getParameter("SetFloatingBaseComponent", _set_floating_base_component)) {
        bfError << "Failed to parse set flaoting base component parameter";
        return false;
    }
    
    if(_set_floating_base_component=="")
    {
        bfError << "NO component selected, please select at least one of it!"; 
       return false;
    }
    
     _gravity_request="";
    if (!m_parameters.getParameter("GravityRequest", _gravity_request)) {
        bfError << "Failed to parse gravity request parameter";
        return false;
    }
    
    if(_gravity_request=="")
    {
        bfError << "NO gravity request, please select at least one of it!"; 
       return false;
    }
    
    _reference_frame="";
    if (!m_parameters.getParameter("ReferenceFrame", _reference_frame)) {
        bfError << "Failed to parse reference frame parameter";
        return false;
    }
    
    // get readings list parameter
    std::string readings_list="";
    if (!m_parameters.getParameter("ReadingsList", readings_list)) {
        bfError << "Failed to parse readings list paramemeter";
        return false;
    }
    
    // convert string to string vector
    CommonUtils::checkParamSelected(readings_list,_readings_list);
    
    // get mechanical limits list parameter
    std::string mechanical_limits_list="";
    if (!m_parameters.getParameter("MechanicalLimitsList", mechanical_limits_list)) {
        bfError << "Failed to parse mechanical limits list paramemeter";
        return false;
    }
    
    // convert string to string vector
    CommonUtils::checkParamSelected(mechanical_limits_list,_mechanical_limits_list);
    
    // get mechanical limits check parameter
    std::string limits_check="";
    if (!m_parameters.getParameter("MechanicalLimitsCheck", limits_check)) {
        bfError << "Failed to parse mechanical limits check paramemeter";
        return false;
    }
    
    if(limits_check=="On")
    {
        _limits_check=true;
    }
    else if(limits_check=="Off")
    {
        _limits_check=false;
    }
    else {
        bfError << "Limits check option:  " << limits_check << " not recognized";
        return false;
    }

    
    // get mechanical limits enforce parameter
    std::string limits_enforce="";
    if (!m_parameters.getParameter("MechanicalLimitsEnforce", limits_enforce)) {
        bfError << "Failed to parse mechanical limits enforce parameter";
        return false;
    }
    
    if(limits_enforce=="On")
    {
        _limits_enforce=true;
    }
    else if(limits_enforce=="Off")
    {
        _limits_enforce=false;
    }
    else {
        bfError << "Limits check option:  " << limits_enforce << " not recognized";
        return false;
    }
    
    // get sensor list parameter
    std::string sensor_list="";
    if (!m_parameters.getParameter("SensorList", sensor_list)) {
        bfError << "Failed to parse sensor list paramemeter";
        return false;
    }
    
    // convert string to string vector
    CommonUtils::checkParamSelected(sensor_list,_sensor_list);
    
    // get references list parameter
    std::string references_list="";
    if (!m_parameters.getParameter("ReferencesList", references_list)) {
        bfError << "Failed to parse references list paramemeter";
        return false;
    }
    
    // convert string to string vector
    CommonUtils::checkParamSelected(references_list,_references_list);
    
    return true;
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_ModelManager::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
{
    // The base blockfactory::core::Block class needs to be configured first
    if (!blockfactory::core::Block::configureSizeAndPorts(blockInfo)) {
       return false;
    }
    
    // read paramemeters of the block
    if (!readParameters(blockInfo)) {
       return false;
    }
    
    std::string error_info = "";
    bool allow_new_model;
    _model_id=0;

    // Store together the port information objects
    blockfactory::core::InputPortsInfo inputPortInfo;
    blockfactory::core::OutputPortsInfo outputPortInfo;

    if(_use_ext_model == UseExtModel::ON)
    {
        // retrieve model
        allow_new_model=false;
        bool model_retrieved = CommonUtils::RetrieveModel(_robot_name,_model_name,allow_new_model,_model_id,_model,error_info);
        
        if(!model_retrieved)
        {
            // avoiding error detection during model retrieving.
            // This operation is done in the configureSizeAndPorts function to detect JOINT NUMBER
            // if there is an error, by default the joint number is equal to 1.
            // Inside the initialize phase will be detected the error.
        }
        
        // Create input for external model id
        blockfactory::core::Port::Info input{/*portIndex=*/0,
                                              std::vector<int>{1},
                                              blockfactory::core::Port::DataType::DOUBLE};
        inputPortInfo.push_back(input);
        
        // avoid setting of floating base of external model
        _set_floating_base_component = "Off";
        // avoid setting of gravity of external model
        if(_gravity_request == "Set" || _gravity_request == "Both")
        {
            bfError << "Cannot set gravity to external model";
            return false; 
        }
    }
    else
    {
        // create model
        allow_new_model=true;
        bool model_created = CommonUtils::RetrieveModel(_robot_name,_model_name,allow_new_model,_model_id,_model,error_info);
        
        // Create output for model id
        blockfactory::core::Port::Info output{/*portIndex=*/0,
                                              std::vector<int>{1},
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
        
        if(!model_created)
        {
            // avoiding error detection during model creation.
            // This operation is done in the configureSizeAndPorts function to detect JOINT NUMBER
            // if there is an error, by default the joint number is equal to 1.
            // Inside the initialize phase will be detected the error.
        }
        
        // Create input for external robot id
        if(_sync_with_robot == SyncWithRobot::ON)
        {
            blockfactory::core::Port::Info input{/*portIndex=*/0,
                                                std::vector<int>{1},
                                                blockfactory::core::Port::DataType::DOUBLE};
            inputPortInfo.push_back(input);
            
        }
    }
    
    // create input/output for set/get floating base
    if(!_get_floating_base_list.empty() || _set_floating_base_component != "Off")
    {
        _floating_base_ptr = std::make_shared<XBB_FloatingBase::XBotBlock_FloatingBase>(_model,
                                                                                        _get_floating_base_list,
                                                                                        _set_floating_base_component,
                                                                                        inputPortInfo.size(),
                                                                                        outputPortInfo.size());
        if(!_floating_base_ptr->configureSizeAndPorts(inputPortInfo,outputPortInfo))
        {
            return false;
        }
    }
    
    // create input/output for set/get gravity
    if(_gravity_request != "Off")
    {
        _gravity_ptr = std::make_shared<XBB_Gravity::XBotBlock_Gravity>(_model,
                                                                        _gravity_request,
                                                                        _reference_frame,
                                                                        inputPortInfo.size(),
                                                                        outputPortInfo.size());
        if(!_gravity_ptr->configureSizeAndPorts(inputPortInfo,outputPortInfo))
        {
            return false;
        }
    }
    
    // create readings class configuring size and ports.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<XBB_Readings::XBotBlock_Readings>(_model,_readings_list,outputPortInfo.size());
        _readings_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    // create mechanical class configuring size and ports.
    if(!_mechanical_limits_list.empty())
    {
        _mechanical_limits_ptr = std::make_shared<XBB_MechanicalLimits::XBotBlock_MechanicalLimits>(_model,_mechanical_limits_list,outputPortInfo.size());
        _mechanical_limits_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    // create sensor manager class configuring size and ports.
    if(!_sensor_list.empty())
    {
        _sensor_manager_ptr = std::make_shared<XBB_SensorManager::XBotBlock_SensorManager>(_model,_sensor_list,outputPortInfo.size());
        _sensor_manager_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    // create references class configuring size and ports.
    if(!_references_list.empty())
    {
        auto mode = XBB_References::XBotBlock_References::Mode::MODEL;
                                                                                         
        _references_ptr = std::make_shared<XBB_References::XBotBlock_References>(_model,
                                                                                 mode,
                                                                                 _references_list,
                                                                                 inputPortInfo.size(),
                                                                                 outputPortInfo.size(),
                                                                                _limits_check,
                                                                                _limits_enforce);
        _references_ptr->configureSizeAndPorts(inputPortInfo,outputPortInfo);
    }
         
    // Store the port information into the BlockInformation
    if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
    }
        
    return true;
}


// Function to perform the sensing on the robot (internal and external) 
// synchronizing it (only with internal robot) with a model 
bool XBotBlock_ModelManager::model_updating()
{
    //check model is a null pointer
    if (_model == nullptr)
    {
        bfError << "Try to perform an operation on a null model";
        return false;
    }
    
    // verify the condition to synchronize the model with the robot
    if(_use_ext_model == UseExtModel::OFF &&
       _sync_with_robot == SyncWithRobot::ON)
    {
        //check robot is a null pointer
        if (_robot != nullptr)
        {
            //synchronization with external robot verifying the errors
            if(!_model->syncFrom(_robot->model()))
            {
                bfError << "Error during the synchronization between model and robot";
                return false;
            }
        }
        else
        {
            bfError << "Try to perform an operation on a null robot";
            return false;
        }
    }
    else
    {
        //update model
        if(!_model->update())
        {
            bfError << "Error during the sense operation";
            return false;
        }
    }
    
    return true;
}

bool XBotBlock_ModelManager::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    // The base blockfactory::core::Block class need to be initialized first
    if (!Block::initialize(blockInfo)) {
        return false;
    }
    
    // read paramemeters of the block
    if (!readParameters(blockInfo)) {
       return false;
    }
    
    std::string error_info;
    bool allow_new_model;
    _model_id=0;
    
    size_t start_input_port=0;
    size_t start_out_port=0;

    if(_use_ext_model == UseExtModel::OFF)
    {
        error_info = "";
        allow_new_model=true;
        // create model and model id
        bool model_created = CommonUtils::RetrieveModel(_robot_name,_model_name,allow_new_model,_model_id,_model,error_info);
        
        // check errors during the creation
        if(!model_created)
        {
            bfError << "Model not created, reason:  " << error_info ;
            return false;
        }
        
        // Get the model id signal
        blockfactory::core::OutputSignalPtr model_id_signal= blockInfo->getOutputPortSignal(/*index=*/0);

        // Check the signal validity
        if (!model_id_signal) 
        {
            bfError << "Model id signal not valid";
            return false;
        }
        //set model id for the signal
        model_id_signal->set(0,(double) _model_id);
        start_out_port=start_out_port+1; // added model ID
        
        if(_sync_with_robot == SyncWithRobot::ON)
        {
            // Get the robot id signal
            blockfactory::core::InputSignalPtr robot_id_signal = blockInfo->getInputPortSignal(/*index=*/0);
            
            // Check the signal validity
            if (!robot_id_signal) {
                bfError << "Robot id signal not valid";
                return false;
            }
            
            // read and casting of uint8_t the robot id signal
            uint8_t robot_id_read = (uint8_t) robot_id_signal->get<double>(0);
            
            // check robot id read validity
            if(!CommonUtils::isValidRobotID(robot_id_read))
            {
                bfError << "robot id read: " << std::to_string(robot_id_read) << " is not a valid robot id";
                return false;
            }
            
            error_info = "";
            bool allow_new_robot=false;
            _robot_id=0;
        
            // retrieve robot and robot id
            bool robot_retrieved = CommonUtils::RetrieveRobot(_robot_name,allow_new_robot,_robot_id,_robot,error_info);
            
            // check errors during robot and robot id retrieving
            if(!robot_retrieved)
            {
                bfError << "Robot not retrieved, reason: " << error_info;
                return false;
            }
            else
            {
                //consistency check between robot id retrieved and robot id read
                if(_robot_id != robot_id_read)
                {
                    bfError << "Robot ID mismatch between the robot name of the block and with the physical signal" << error_info;
                    return false;
                }
            }
        
            //consistency check between model and robot configuration
            if(!_model->getConfigOptions().is_same_robot(_robot->getConfigOptions()))
            {
                bfError << "Model has a different configuration file of external robot";
                return false;
            }
        
            start_input_port = start_input_port + 1; //added robot ID
        }
    }
    else
    {
        // Get the model id signal
        blockfactory::core::InputSignalPtr model_id_signal = blockInfo->getInputPortSignal(/*index=*/0);
        
        // Check the signal validity
        if (!model_id_signal) {
            bfError << "Model id signal not valid";
            return false;
        }
        
        // read and casting of uint8_t the model id signal
        uint8_t model_id_read = (uint8_t) model_id_signal->get<double>(0);
        
        // check model id read validity
        if(!CommonUtils::isValidModelID(model_id_read))
        {
            bfError << "model id read: " << std::to_string(model_id_read) << " is not a valid model id";
            return false;
        }
        
        error_info = "";
        allow_new_model=false;
        // retrieve model and model id
        bool model_retrieved = CommonUtils::RetrieveModel(_robot_name,_model_name,allow_new_model,_model_id,_model,error_info);
        
        // check errors during model and model id retrieving
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
        }
        
        start_input_port = start_input_port + 1; //added external model ID
        
        // avoid setting of floating base of external model
        _set_floating_base_component = "Off";
    }
    
    // first update to read actual value
    if(!model_updating())
    {
        return false;
    }
    
    if(!_get_floating_base_list.empty() || _set_floating_base_component != "Off")
    {
        _floating_base_ptr = std::make_shared<XBB_FloatingBase::XBotBlock_FloatingBase>(_model,
                                                                                        _get_floating_base_list,
                                                                                        _set_floating_base_component,
                                                                                        start_input_port,
                                                                                        start_out_port);
        if(!_floating_base_ptr->initialize(blockInfo))
        {
            return false;
        }
        
        // count the start out port for readings port.
        start_out_port = start_out_port + _get_floating_base_list.size();
        
        if(_set_floating_base_component != "Off")
        {
            // add homogeneous matrix port
            if(_set_floating_base_component != "State")
            {
                start_input_port = start_input_port + 1;
            }
            else
            {
                start_input_port = start_input_port + 2;
            }
                
        }
    }
    
    // create input/output for set/get gravity
    if(_gravity_request != "Off")
    {
        _gravity_ptr = std::make_shared<XBB_Gravity::XBotBlock_Gravity>(_model,
                                                                        _gravity_request,
                                                                        _reference_frame,
                                                                        start_input_port,
                                                                        start_out_port);
        
        if(!_gravity_ptr->initialize(blockInfo))
        {
            return false;
        }
        
        if((_gravity_request =="Get")||(_gravity_request =="Both"))
        {
            start_out_port = start_out_port + 1;
        }
        
        if((_gravity_request =="Set")||(_gravity_request =="Both"))
        {
            start_input_port = start_input_port + 1;
        }
    }
    
    // readings creation and initialization with initial update.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<XBB_Readings::XBotBlock_Readings>(_model,_readings_list,start_out_port);
        if(!_readings_ptr->initialize(blockInfo))
        {
            return false;
        }
    }
    
    // count the start out port for mechanical limits port.
    start_out_port = start_out_port + _readings_list.size();
    // readings mechanical limits creation and initialization.
    if(!_mechanical_limits_list.empty())
    {
        _mechanical_limits_ptr = std::make_shared<XBB_MechanicalLimits::XBotBlock_MechanicalLimits>(_model,_mechanical_limits_list,start_out_port);
        if(!_mechanical_limits_ptr->initialize(blockInfo))
        {
            return false;
        }
    }
    
    // count the start out port the write the right mechanical limits port.
    start_out_port = start_out_port + _mechanical_limits_list.size();
    // readings sensor ids  creation and initialization.
    if(!_sensor_list.empty())
    {
        _sensor_manager_ptr = std::make_shared<XBB_SensorManager::XBotBlock_SensorManager>(_model,_sensor_list,start_out_port);
        
        if(!_sensor_manager_ptr->initialize(blockInfo))
        {
            return false;
        }
    }
    
    // count the start out port the write the right mechanical limits port.
    start_out_port = start_out_port + _sensor_list.size();
    // references creation and initialization.
    if(!_references_list.empty())
    {
        // MODE is necessary because XBotInterface library has different methods to set the references from robot and model.
        auto mode = XBB_References::XBotBlock_References::Mode::MODEL;
        _references_ptr = std::make_shared<XBB_References::XBotBlock_References>(_model,
                                                                                 mode,
                                                                                 _references_list,
                                                                                 start_input_port,
                                                                                 start_out_port,
                                                                                 _limits_check,
                                                                                 _limits_enforce);
    }

    return true;
}

bool XBotBlock_ModelManager::output(const blockfactory::core::BlockInformation* blockInfo)
{
    // perform updating operation
    
    // NOTE: updating operation is done even if only mechanical limits are requested by the users.
    //       This delete duplications of model managers.

    if(! model_updating())
    {
        return false;
    }
    
    // get and set floating base (redundant model update)
    if(!_get_floating_base_list.empty() || _set_floating_base_component != "Off")
    {
        if(!_floating_base_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    // get and set gravity (redundant model update)
    if(_gravity_request != "Off")
    {
        if(!_gravity_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    // get model input
    if(_readings_ptr != nullptr)
    {
        if(!_readings_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    // NOTE: no getting mechanical limits operation during this phase, 
    //       since already done during one time on the initialization phase. 
    
    // set model output 
    
    // NOTE: output delayed of one simple time due of first model model_updating 
    if(_references_ptr != nullptr)
    {
        if(!_references_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    return true;
    
}

bool XBotBlock_ModelManager::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    // clear robot and model maps
    CommonUtils::clearRobotMap();
    CommonUtils::clearModelMap();
    return true;
    
}
