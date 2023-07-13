#include "manager/robot_manager.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM_NUM 2

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
    std::string name[PARAM_NUM] = {"ReadingsList",
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
   
    // get readings list parameter
    std::string readings_list="";
    if (!m_parameters.getParameter("ReadingsList", readings_list)) {
        bfError << "Failed to parse readings list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::check_param_selected(readings_list,_readings_list);
    

    
    // get references list parameter
    std::string references_list="";
    if (!m_parameters.getParameter("ReferencesList", references_list)) {
        bfError << "Failed to parse references list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::check_param_selected(references_list,_references_list);
    
    
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
    
    // Store together the port information objects
    blockfactory::core::InputPortsInfo inputPortInfo;
    blockfactory::core::OutputPortsInfo outputPortInfo;
    
    // create readings class configuring size and ports.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<EcBlock::Reading>(_robot,_readings_list,outputPortInfo.size());
        _readings_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    
    // create references class configuring size and ports.
    if(!_references_list.empty())
    {
        _references_ptr = std::make_shared<EcBlock::Reference>(_robot,_references_list,inputPortInfo.size());
        _references_ptr->configureSizeAndPorts(inputPortInfo);
    }
         
    // Store the port information into the BlockInformation
    if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
    }
        
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
    
    std::string error_info="";
    
    size_t start_input_port=0;
    size_t start_out_port=0;
    
    _do_move = _avoid_first_move = false;
    // retrieve robot
    bool robot_retrieved = EcBlockUtils::retrieve_robot(_robot,error_info);
    
    // check errors during the creation
    if(!robot_retrieved)
    {
        bfError << "Robot not retrieved, reason: " << error_info;
        return false;
    }
    
    // readings creation and initialization with initial sense.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<EcBlock::Reading>(_robot,_readings_list,start_out_port);

        // first sense to read actual value
        _motors_status_map = EcBlockUtils::robot_sensing();
        if(!_readings_ptr->initialize(blockInfo,_motors_status_map))
        {
            return false;
        }
        
        // unit delay during sensing and moving operation
        _avoid_first_move=true;
    }
    
    
    // references creation and initialization.
    if(!_references_list.empty())
    {
        // MODE is necessary because XBotInterface library has different methods to set the references from robot and model.
        _references_ptr = std::make_shared<EcBlock::Reference>(_robot,_references_list,start_input_port);
        
        _do_move = true;
    }
         
    return true;
}

bool RobotManager::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(!_robot->is_client_alive())
    {
        bfError << "EtherCAT Client not alive! ";
        return false;
    }
    // get robot input
    if(_readings_ptr != nullptr)
    {
        // perform sensing operation
        _motors_status_map.clear();
        _motors_status_map = EcBlockUtils::robot_sensing();
        if(!_readings_ptr->output(blockInfo,_motors_status_map))
        {
            return false;
        }
    }
    
    // set robot output
    // added avoid first move check in order to avoid to set wrong references.
    if(_references_ptr != nullptr && !_avoid_first_move)
    {
        _motors_ref = EcBlockUtils::retrieve_motors_ref();
        if(!_references_ptr->output(blockInfo,_motors_ref))
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
            _robot->set_motors_references(MotorRefFlags::FLAG_MULTI_REF,_motors_ref);
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
    EcBlockUtils::clear_robot();
    EcBlockUtils::clearModelMap();
    return true;
    
}
