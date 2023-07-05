#include "XBotBlock/Sensors/ForceTorque.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

#define PARAM 3

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(XBotBlock_ForceTorque, XBB_ForceTorque::XBotBlock_ForceTorque, blockfactory::core::Block);

using Matrix6x1Simulink = Eigen::Matrix<double, 6, 1, Eigen::ColMajor>;

using namespace XBB_ForceTorque;


using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

unsigned XBotBlock_ForceTorque::numberOfParameters()
{
    // The base blockfactory::core::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + PARAM;
}

// This method should let BlockInformation know the parameters metadata.
// BlockFactory will use this information to gather the parameters from the active engine.
bool XBotBlock_ForceTorque::parseParameters(blockfactory::core::BlockInformation* blockInfo)
{ 
    // Initialize information for our parameter
    int rows = 1;
    int cols = 1;
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    std::string name[PARAM] = {"ManagerName","ManagerType","ForceTorqueList"}; // This label is used later to access the paramemeter
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

bool XBotBlock_ForceTorque::readParameters(blockfactory::core::BlockInformation* blockInfo)
{
    // Parse the parameters
    if (!XBotBlock_ForceTorque::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }
    
   // get model name parameter
    _manager_name="";
    if (!m_parameters.getParameter("ManagerName", _manager_name)) {
        bfError << "Failed to parse manager name parameter";
        return false;
    }   
    
    if(_manager_name=="")
    {
        bfError << "Missing robot or model name";
        return false;
    }
    
    // get single manager type parameter
    _manager_type="";
    if (!m_parameters.getParameter("ManagerType", _manager_type)) {
        bfError << "Failed to parse manager type parameter";
        return false;
    }
    
    if(_manager_type=="")
    {
        bfError << "Manager type parameter cannot be empty";
        return false;
    }
    
    // get force torque list parameter
    std::string force_torque_list;
    if (!m_parameters.getParameter("ForceTorqueList", force_torque_list)) {
        bfError << "Failed to parse Force Torque list";
        return false;
    }
    
    CommonUtils::checkParamSelected(force_torque_list,_force_torque_list);
    
    if(_force_torque_list.empty())
    {
       bfError << "NO param selected, please select at least one of it!"; 
       return false;
    }

    return true;
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_ForceTorque::configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo)
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
    
    blockfactory::core::Port::Info manager_id_in_port{/*portIndex=*/0,
                                                      std::vector<int>{1},
                                                      blockfactory::core::Port::DataType::DOUBLE};

    inputPortInfo.push_back(manager_id_in_port);  
    
    blockfactory::core::Port::Info ft_id_in_port{/*portIndex=*/1,
                                                 std::vector<int>{1},
                                                 blockfactory::core::Port::DataType::DOUBLE};

    inputPortInfo.push_back(ft_id_in_port);  

    
    for(size_t i=0; i < _force_torque_list.size();i++)
    {
        std::vector<int> port_size;
        if(_force_torque_option.count(_force_torque_list[i]) > 0)
        {
            switch(_force_torque_option.at(_force_torque_list[i]))
            {
                case f: {
                            port_size.push_back(3);
                        }break;
                        
                case t: {
                            port_size.push_back(3);
                        }break;
                            
                case w: {
                            port_size.push_back(1);
                            port_size.push_back(6);
                        }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized force torque option from parameters selected";
            return false;
        }
        
        size_t index = outputPortInfo.size();
        
        blockfactory::core::Port::Info output{/*portIndex=*/index,
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

bool XBotBlock_ForceTorque::setForceTorqueOut(const blockfactory::core::BlockInformation* blockInfo)
{
    //check force torque is a null pointer
    if (_force_torque != nullptr)
    {
        for(size_t i=0; i < _force_torque_list.size();i++)
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i );
            // Check the signal validity
            if (!output) {
                bfError << "Signal not valid";
                return false;
            }
            // verify if the element of the list exists like option
            if(_force_torque_option.count(_force_torque_list[i]) > 0)
            {
                // save into auxiliary vector the ForceTorque information
                Eigen::Vector3d aux_vector;
                switch(_force_torque_option.at(_force_torque_list[i]))
                {
                    case f: {
                                _force_torque->getForce(aux_vector);   
                            }break;
                    case t: {
                                _force_torque->getTorque(aux_vector);   
                            }break;
                    case w: {
                                Eigen::Matrix<double, 6, 1> wrench;
                                _force_torque->getWrench(wrench);
                                // Allocate objects for row-major -> col-major conversion
                                Eigen::Map<Matrix6x1Simulink> WrenchMajor(output->getBuffer<double>(),6,1);
                                WrenchMajor=wrench.matrix();
                            }break;             
                }
                if(_force_torque_list[i] != "w") 
                {
                    // check the auxiliary vector size with output signal size
                    if(aux_vector.size() != output->getWidth())
                    {
                        bfError << "Different dimension of output port: " + std::to_string(output->getWidth()) +" and force torque vector: " + std::to_string(aux_vector.size());
                        return false;
                    }
                    
                    // set the signal output
                    for (size_t  k=0; k < output->getWidth(); ++k)
                    {
                        output->set(k, aux_vector[k]);
                    }
                }
            }
            else
            {
                bfError << "Found unrecognized imu components option from parameters selected";
                return false;
            }
        }
    }
    else
    {
        bfError << "Try to perform an operation on a null force torque, please verify the sensor ID";
        return false;
    }

    return true;
}


bool XBotBlock_ForceTorque::initialize(blockfactory::core::BlockInformation* blockInfo)
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
    blockfactory::core::InputSignalPtr manager_id_signal = blockInfo->getInputPortSignal(/*index=*/0);
    
    // Check the signal validity
    if (!manager_id_signal) {
        bfError << _manager_type + " id signal not valid";
        return false;
    }
    
    // retrieve robot/model and robot_id/model id
    std::string error_info = "";
    
    if(_manager_type=="Robot")
    {
        // read and casting of uint8_t the robot id signal
        uint8_t robot_id_read = (uint8_t) manager_id_signal->get<double>(0);
        
        // check robot id read validity
        if(!CommonUtils::isValidRobotID(robot_id_read))
        {
            bfError << "robot id read: " << std::to_string(robot_id_read) << " is not a valid robot id";
            return false;
        }

        bool allow_new_robot=false;
        _robot_id=0;
    
        // retrieve robot and robot id
        bool robot_retrieved = CommonUtils::RetrieveRobot(_manager_name,allow_new_robot,_robot_id,_robot,error_info);
        
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
    }
    else
    {
         uint8_t model_id_read = (uint8_t) manager_id_signal->get<double>(0);
        
        // check model id read validity
        if(!CommonUtils::isValidModelID(model_id_read))
        {
            bfError << "model id read: " << std::to_string(model_id_read) << " is not a valid model id";
            return false;
        }
        
        // retrieve model and model id
        error_info = "";
        bool allow_new_model=false;
        _model_id=0;
        bool model_retrieved = CommonUtils::RetrieveModel("",_manager_name,allow_new_model,_model_id,_model,error_info);
        
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
    }
    
    return true;
}

bool XBotBlock_ForceTorque::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(_force_torque==nullptr)
    {
        // Get the sensor id signal
        blockfactory::core::InputSignalPtr sensor_id_signal = blockInfo->getInputPortSignal(/*index=*/1);
        
        // Check the signal validity
        if (!sensor_id_signal) {
            bfError << "Sensor id signal not valid";
            return false;
        }
        
        int sensor_id_read = (int) sensor_id_signal->get<double>(0);

        if(_manager_type=="Robot")
        {
            // get ForceTorque using the ID
            _force_torque = _robot->getForceTorque(sensor_id_read);
        }
        else
        {
            // get ForceTorque using the ID
            _force_torque = _model->getForceTorque(sensor_id_read);
        }
    }
    
    if(!setForceTorqueOut(blockInfo))
    {
        return false;
    }
    
    return true;
    
}

bool XBotBlock_ForceTorque::terminate(const blockfactory::core::BlockInformation* /*blockInfo*/)
{
    CommonUtils::clearModelMap();
    return true;
    
}
