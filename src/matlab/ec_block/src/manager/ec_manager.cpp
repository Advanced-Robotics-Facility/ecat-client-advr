#include "manager/ec_manager.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM_NUM 5

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(EcManager, EcBlock::EcManager, blockfactory::core::Block);

using namespace EcBlock;

unsigned EcManager::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM_NUM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool EcManager::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM_NUM] = {"ReadingsList",
                                   "ReferencesList",
                                   "ImuList",
                                   "FtList",
                                   "PowList"}; // This label is used later to access the paramemeter
                                   
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

bool EcManager::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!EcManager::parseParameters(blockInfo)) {
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

    
    // get  Imu list parameter
    std::string imu_list="";
    if (!m_parameters.getParameter("ImuList", imu_list)) {
        bfError << "Failed to parse imu list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::check_param_selected(imu_list,_imu_list);
    
    
    // get Ft list parameter
    std::string ft_list="";
    if (!m_parameters.getParameter("FtList", ft_list)) {
        bfError << "Failed to parse ft list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::check_param_selected(ft_list,_ft_list);
    
    // get Pow list parameter
    std::string pow_list="";
    if (!m_parameters.getParameter("PowList", pow_list)) {
        bfError << "Failed to parse pow list paramemeter";
        return false;
    }
    
    // convert string to string vector
    EcBlockUtils::check_param_selected(pow_list,_pow_list);
    
    
    
    if(_readings_list.empty() && 
        _references_list.empty() && 
        _imu_list.empty() && 
        _ft_list.empty() &&
        _pow_list.empty())
    {
        bfError << "NO param selected, please select at least one of it!"; 
        return false;
    }
    
    return true;
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool EcManager::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
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
        _readings_ptr = std::make_shared<EcBlock::Reading>(_readings_list,outputPortInfo.size());
        _readings_ptr->configureSizeAndPorts(outputPortInfo);
    }
    
    // create imu class configuring size and ports.
    if(!_imu_list.empty())
    {
        _imu_ptr = std::make_shared<EcBlock::Imu>(_imu_list,outputPortInfo.size());
        if(!_imu_ptr->configureSizeAndPorts(outputPortInfo))
        {
            return false;
        }
    }
    
    // create ft class configuring size and ports.
    if(!_ft_list.empty())
    {
        _ft_ptr = std::make_shared<EcBlock::Ft>(_ft_list,outputPortInfo.size());
        if(!_ft_ptr->configureSizeAndPorts(outputPortInfo))
        {
            return false;
        }
    }
    
    // create pow class configuring size and ports.
    if(!_pow_list.empty())
    {
        _pow_ptr = std::make_shared<EcBlock::Pow>(_pow_list,outputPortInfo.size());
        if(!_pow_ptr->configureSizeAndPorts(outputPortInfo))
        {
            return false;
        }
    }
    
    // create references class configuring size and ports.
    if(!_references_list.empty())
    {
        _references_ptr = std::make_shared<EcBlock::Reference>(_references_list,inputPortInfo.size());
        _references_ptr->configureSizeAndPorts(inputPortInfo);
    }
         
    // Store the port information into the BlockInformation
    if (!blockInfo->setPortsInfo(inputPortInfo, outputPortInfo)) {
        bfError << "Failed to configure input / output ports";
        return false;
    }
        
    return true;
}


bool EcManager::initialize(blockfactory::core::BlockInformation* blockInfo)
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
    bool start_robot_req=false;
    if(!_readings_list.empty() || !_references_list.empty())
    {
        start_robot_req=true;
    }
    // retrieve ec
    bool ec_retrieved = EcBlockUtils::retrieve_ec_iface(error_info,start_robot_req);
    
    // check errors during the creation
    if(!ec_retrieved)
    {
        bfError << "EtherCAT Client not retrieved, reason: " << error_info;
        return false;
    }
    
    // first sense to read actual value
    if(!EcBlockUtils::ec_sense(_motors_status_map,_ft_status_map,_imu_status_map,_pow_status_map,_motors_ref))
    {
        bfError << "EtherCAT Client not alive! ";
        return false;
    }
    
    // readings creation and initialization with initial sense.
    if(!_readings_list.empty())
    {
        _readings_ptr = std::make_shared<EcBlock::Reading>(_readings_list,start_out_port);

        if(!_readings_ptr->initialize(blockInfo,_motors_status_map,_motors_ref))
        {
            return false;
        }
        // unit delay during sensing and moving operation
        _avoid_first_move=true;
    }
    
    start_out_port = start_out_port + _readings_list.size();
    
    // imu creation and initialization with initial sense.
    if(!_imu_list.empty())
    {
        _imu_ptr = std::make_shared<EcBlock::Imu>(_imu_list,start_out_port);

        if(!_imu_ptr->initialize(blockInfo,_imu_status_map))
        {
            return false;
        }
    }
    
    start_out_port = start_out_port + _imu_list.size();
    
    
    // ft creation and initialization with initial sense.
    if(!_ft_list.empty())
    {
        _ft_ptr = std::make_shared<EcBlock::Ft>(_ft_list,start_out_port);

        if(!_ft_ptr->initialize(blockInfo,_ft_status_map))
        {
            return false;
        }
    }
    
    start_out_port = start_out_port + _ft_list.size();
    
    // pow creation and initialization with initial sense.
    if(!_pow_list.empty())
    {
        _pow_ptr = std::make_shared<EcBlock::Pow>(_pow_list,start_out_port);

        if(!_pow_ptr->initialize(blockInfo,_pow_status_map))
        {
            return false;
        }
    }
    
    
    // references creation and initialization.
    if(!_references_list.empty())
    {
        _references_ptr = std::make_shared<EcBlock::Reference>(_references_list,start_input_port);
        
        _do_move = true;
    }
         
    return true;
}

bool EcManager::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(!EcBlockUtils::ec_sense(_motors_status_map,_ft_status_map,_imu_status_map,_pow_status_map,_motors_ref))
    {
        bfError << "EtherCAT Client not alive! ";
        return false;
    }
    
    // get ec input
    if(_readings_ptr != nullptr)
    {
        if(!_readings_ptr->output(blockInfo,_motors_status_map,_motors_ref))
        {
            return false;
        }
    }
    
    // get imu input
    if(_imu_ptr != nullptr)
    {
        if(!_imu_ptr->output(blockInfo,_imu_status_map))
        {
            return false;
        }
    }
    
    // get ft input
    if(_ft_ptr != nullptr)
    {
        if(!_ft_ptr->output(blockInfo,_ft_status_map))
        {
            return false;
        }
    }
    
    // get pow input
    if(_pow_ptr != nullptr)
    {
        if(!_pow_ptr->output(blockInfo,_pow_status_map))
        {
            return false;
        }
    }
    
    // set ec output
    // added avoid first move check in order to avoid to set wrong references.
    if(_references_ptr != nullptr && !_avoid_first_move)
    {
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
            if(!EcBlockUtils::ec_move(RefFlags::FLAG_MULTI_REF,_motors_ref))
            {
                bfError << "EtherCAT Client not alive! ";
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

bool EcManager::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    // stop ec
    EcBlockUtils::stop_ec_iface();
    return true;
    
}
