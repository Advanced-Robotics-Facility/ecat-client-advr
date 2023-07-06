#include "manager/reference.h"
#include "ec_block_utils.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;

Reference::Reference(XBot::XBotInterface::Ptr xbi,
                     Mode mode,
                     std::vector<std::string> references_list,
                     size_t in_start_port,
                     size_t out_start_port,
                     bool limits_check,
                     bool limits_enforce):
_xbi(xbi),
_mode(mode),
_references_list(references_list),
_in_start_port(in_start_port),
_out_start_port(out_start_port),
_limits_check(limits_check),
_limits_enforce(limits_enforce)
{
    // create class and save the XBotInterface ptr, list and start port (input and output)
    // initialize joint_number to one
    _joint_number = 1;
    
    //override joint_number
    if(_xbi != nullptr)
    {
        _joint_number = _xbi->getJointNum();
    }
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
void Reference::configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                                                 blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    size_t ports = _references_list.size();
    
    size_t index_check_limits=0;
    
    // configure inputPortInfo
    for(size_t i=0; i < ports;i++)
    {
        blockfactory::core::Port::Info input{/*portIndex=*/i + _in_start_port,
                                              std::vector<int>{_joint_number},
                                              blockfactory::core::Port::DataType::DOUBLE};
        inputPortInfo.push_back(input);
       
        if(_references_list[i]=="qJ_ref" || _references_list[i]=="qJdot_ref" || _references_list[i]=="tau_ref")
        {
            if(_limits_check)
            {
                blockfactory::core::Port::Info output{/*portIndex=*/_out_start_port + index_check_limits,
                                                      std::vector<int>{1},
                                                      blockfactory::core::Port::DataType::DOUBLE};
                
                outputPortInfo.push_back(output);                                      
                index_check_limits++;
            }
        }
    }
}


bool Reference::setReferences(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info)
{
    size_t index_check_limits=0;
    // set all ouput of the list
    for(size_t i=0; i < _references_list.size();i++)
    {
        // get input signal
        blockfactory::core::InputSignalPtr input = blockInfo->getInputPortSignal(/*index=*/ i + _in_start_port);
        // Check the signal validity
        if (!input) {
            error_info = "Signal not valid";
            return false;
        }
        
        // check dimesion of the input signal with the joint number
        // NOTE this allow to avoid the checks (bool return) on setting function
        if(input->getWidth() != _joint_number)
        {
            error_info = "Different dimension of input port: " + std::to_string(input->getWidth()) +" and joint number: " + std::to_string(_joint_number);
            return false;
        }
        
        // use an auxiliary vectory to save the input signal values
        Eigen::VectorXd aux_vector;
        aux_vector.resize(_joint_number);
        
        for (size_t  k=0; k < aux_vector.size(); ++k)
        {
            aux_vector[k]=input->get<double>(k);
        }
        
        // verify if the element of the list exists like option
        if(_references_options.count(_references_list[i]) > 0)
        {
            // set aux_vector like reference 
            switch(_references_options.at(_references_list[i]))
            {
                case qJ_ref: {             
                                if(_limits_enforce)
                                {
                                    _xbi->enforceJointLimits(aux_vector);
                                }
                                if(_limits_check)
                                {
                                    // get ouput signal
                                    blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/_out_start_port +index_check_limits);
                                    // Check the signal validity
                                    if (!output) {
                                        error_info = "Signal not valid";
                                        return false;
                                    }
                                    
                                    bool q_limits_check = _xbi->checkJointLimits(aux_vector);

                                    output->set(0, q_limits_check);
                                    
                                    index_check_limits++;
                                }
                                
                                if(_mode==Mode::ROBOT)
                                {
                                    _xbi->setPositionReference(aux_vector); 
                                }
                                else
                                {
                                    _xbi->setJointPosition(aux_vector);
                                }
                            }break;
                case qM_ref: {
                                if(_mode==Mode::ROBOT)
                                {
                                        error_info = "Cannot set motor position reference to a robot";
                                        return false;
                                }
                                else
                                {
                                    _xbi->setMotorPosition(aux_vector);
                                }
                            }break;            
                            
                case qJdot_ref:  {
                                    if(_limits_enforce)
                                    {
                                        _xbi->enforceVelocityLimit(aux_vector);
                                    }
                                    
                                    if(_limits_check)
                                    {
                                        // get ouput signal
                                        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/_out_start_port +index_check_limits);
                                        // Check the signal validity
                                        if (!output) {
                                            error_info = "Signal not valid";
                                            return false;
                                        }
                                        
                                        bool qdot_limits_check = _xbi->checkVelocityLimits(aux_vector);
                                        
                                        output->set(0, qdot_limits_check);
                                        
                                        index_check_limits++;
                                    }
                                    if(_mode==Mode::ROBOT)
                                    {
                                        _xbi->setVelocityReference(aux_vector); 
                                    }
                                    else
                                    {
                                        _xbi->setJointVelocity(aux_vector);
                                    }
                                }break;   
                
                case qMdot_ref: {
                                if(_mode==Mode::ROBOT)
                                {
                                        error_info = "Cannot set motor velocity reference to a robot";
                                        return false;
                                }
                                else
                                {
                                    _xbi->setMotorVelocity(aux_vector);
                                }
                            }break; 
                
                case qJddot_ref: {
                                    if(_mode==Mode::ROBOT)
                                    {
                                        error_info = "Cannot set joint acceleration reference to a robot";
                                        return false;
                                    }
                                    else
                                    {
                                        _xbi->setJointAcceleration(aux_vector);
                                    }
                                }break;
                case tau_ref:   {
                                    if(_limits_enforce)
                                    {
                                        _xbi->enforceEffortLimit(aux_vector);
                                    }
                                    
                                    if(_limits_check)
                                    {
                                        // get ouput signal
                                        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/_out_start_port +index_check_limits);
                                        // Check the signal validity
                                        if (!output) {
                                            error_info = "Signal not valid";
                                            return false;
                                        }

                                        bool tau_limits_check = _xbi->checkEffortLimits(aux_vector);
                                        
                                        output->set(0, tau_limits_check);
                                        
                                        index_check_limits++;
                                    }
                                    
                                    if(_mode==Mode::ROBOT)
                                    {
                                        _xbi->setEffortReference(aux_vector);
                                    }
                                    else
                                    {
                                        _xbi->setJointEffort(aux_vector);
                                    }
                                }break;
                case qTemp_ref: {
                                    _xbi->setTemperature(aux_vector);
                                }break;
                case K_ref: {
                                _xbi->setStiffness(aux_vector);
                            }break;
                case D_ref: {
                                _xbi->setDamping(aux_vector);
                            }break;
                
            }
        }
        else
        {
            error_info = "Found unrecognized joint references option from parameters selected";
            return false;
        }
    }
    return true;
}

// NOTE: only output fuction was developed for the references, the intialization phase is not needed.

bool Reference::output(const blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    if(!setReferences(blockInfo,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }
    return true;    
}
