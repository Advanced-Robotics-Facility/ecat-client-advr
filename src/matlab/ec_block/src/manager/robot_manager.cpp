#include "manager/robot_manager.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM_NUM 3

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(RobotManager, EcBlock::RobotManager, blockfactory::core::Block);

using namespace EcBlock;

unsigned RobotManager::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM_NUM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool RobotManager::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM_NUM] = {"UseExtRobot",
                                   "ReadingsList",
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

bool RobotManager::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!RobotManager::parseParameters(blockInfo)) {
        bfError <<  "Failed to parse parameters.";
        return false;
    }
    // get use external robot parameter
    std::string use_ext_robot="";
    if (!m_parameters.getParameter("UseExtRobot", use_ext_robot)) {
        bfError << "Failed to parse use external robot paramemeter";
        return false;
    }
    
    // Check the content of the parameter
    if (use_ext_robot == "Off") {
        _use_ext_robot = UseExtRobot::OFF;
    }
    else if (use_ext_robot == "On") {
        _use_ext_robot = UseExtRobot::ON;
    }
    else {
        bfError << "Use external option:  " << use_ext_robot << " not recognized";
        return false;
    }
    

    // get readings list parameter
    std::string readings_list="";
    if (!m_parameters.getParameter("ReadingsList", readings_list)) {
        bfError << "Failed to parse readings list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::checkParamSelected(readings_list,_readings_list);
    

    
    // get references list parameter
    std::string references_list="";
    if (!m_parameters.getParameter("ReferencesList", references_list)) {
        bfError << "Failed to parse references list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::checkParamSelected(references_list,_references_list);
    
    
    return true;
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool RobotManager::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
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
    bool allow_new_robot;

    // Store together the port information objects
    blockfactory::core::InputPortsInfo inputPortInfo;
    blockfactory::core::OutputPortsInfo outputPortInfo;

    if(_use_ext_robot == UseExtRobot::ON)
    {
        // retrieve robot
        allow_new_robot=false;
        bool robot_retrieved = EcBlockUtils::RetrieveRobot(allow_new_robot,_robot_new,error_info);
        
        if(!robot_retrieved)
        {
            // avoiding error detection during robot retrieving.
            // This operation is done in the configureSizeAndPorts function to detect JOINT NUMBER
            // if there is an error, by default the joint number is equal to 1.
            // Inside the initialize phase will be detected the error.
        }
    }
    else
    {
        // create robot
        allow_new_robot=true;
        bool robot_created = EcBlockUtils::RetrieveRobot(allow_new_robot,_robot_new,error_info);
        
        if(!robot_created)
        {
            // avoiding error detection during robot creation.
            // This operation is done in the configureSizeAndPorts function to detect JOINT NUMBER
            // if there is an error, by default the joint number is equal to 1.
            // Inside the initialize phase will be detected the error.
        }
    }
    
    // create readings class configuring size and ports.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<EcBlock::Reading>(_robot,_readings_list,outputPortInfo.size());
        _readings_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    
    // create references class configuring size and ports.
    if(!_references_list.empty())
    {
        auto mode = EcBlock::Reference::Mode::ROBOT;
        _references_ptr = std::make_shared<EcBlock::Reference>(_robot,
                                                                                 mode,
                                                                                 _references_list,
                                                                                 inputPortInfo.size(),
                                                                                 outputPortInfo.size(),
                                                                                false,
                                                                                false);
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
bool RobotManager::robot_sensing()
{
    
    return true;
}

bool RobotManager::initialize(blockfactory::core::BlockInformation* blockInfo)
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
    bool allow_new_robot;
    
    size_t start_input_port=0;
    size_t start_out_port=0;
    
    _do_sense = _do_move = _avoid_first_move = false;
    
    if(_use_ext_robot == UseExtRobot::OFF)
    {
        error_info = "";
        allow_new_robot=true;
        // create robot and robot id
        bool robot_created = EcBlockUtils::RetrieveRobot(allow_new_robot,_robot_new,error_info);
        
        // check errors during the creation
        if(!robot_created)
        {
            bfError << "Robot not created, reason: " << error_info;
            return false;
        }
    }
    else
    {
        
        error_info = "";
        allow_new_robot=false;
        // retrieve robot and robot id
        bool robot_retrieved = EcBlockUtils::RetrieveRobot(allow_new_robot,_robot_new,error_info);
        
        // check errors during robot and robot id retrieving
        if(!robot_retrieved)
        {
            bfError << "Robot not retrieved, reason: " << error_info;
            return false;
        }
    }
    
    // readings creation and initialization with initial sense.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<EcBlock::Reading>(_robot,_readings_list,start_out_port);
        
        // first sense to read actual value
        if(!robot_sensing())
        {
            return false;
        }

        if(!_readings_ptr->initialize(blockInfo))
        {
            return false;
        }
        
        _do_sense = true;
        
        // unit delay during sensing and moving operation
        _avoid_first_move=true;
    }
    
    // count the start out port the write the right mechanical limits port.
    start_out_port = start_out_port + _readings_list.size();
    
    
    // references creation and initialization.
    if(!_references_list.empty())
    {
        // MODE is necessary because XBotInterface library has different methods to set the references from robot and model.
        auto mode = EcBlock::Reference::Mode::ROBOT;
        _references_ptr = std::make_shared<EcBlock::Reference>(_robot,
                                                                                 mode,
                                                                                 _references_list,
                                                                                 start_input_port,
                                                                                 start_out_port,
                                                                                 false,
                                                                                 false);
        
        _do_move = true;
    }
         
    return true;
}

bool RobotManager::output(const blockfactory::core::BlockInformation* blockInfo)
{
    // perform sensing operation
    if(_do_sense)
    {
        if(!robot_sensing())
        {
            return false;
        }
    }

    // get robot input
    if(_readings_ptr != nullptr)
    {
        if(!_readings_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    // NOTE: no getting mechanical limits operation during this phase, 
    //       since already done during one time on the initialization phase. 
    
    // set robot output
    // added avoid first move check in order to avoid to set wrong references.
    if(_references_ptr != nullptr && !_avoid_first_move)
    {
        if(!_references_ptr->output(blockInfo))
        {
            return false;
        }
    }
    
    // perform moving operation
    if(_do_move)
    {
        // unit delay during sensing and moving operation
        if(!_avoid_first_move)
        {
            //move robot
            if(!_robot->move())
            {
                bfError << "Error during the move operation";
                return false;
            }
        }
        else
        {
            _avoid_first_move=false;
        }
    }
    
    return true;
    
}

bool RobotManager::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    // clear robot and model map
    EcBlockUtils::clearRobot();
    EcBlockUtils::clearModelMap();
    return true;
    
}
