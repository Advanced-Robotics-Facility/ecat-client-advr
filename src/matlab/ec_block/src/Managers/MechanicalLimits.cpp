#include "XBotBlock/Managers/MechanicalLimits.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>


using namespace XBB_MechanicalLimits;

XBotBlock_MechanicalLimits::XBotBlock_MechanicalLimits(XBot::XBotInterface::Ptr xbi,
                                                       std::vector<std::string> mechanical_limits_list,
                                                       size_t start_port):
_xbi(xbi),
_mechanical_limits_list(mechanical_limits_list),
_start_port(start_port)
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
void XBotBlock_MechanicalLimits::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    size_t ports = _mechanical_limits_list.size();
    
    // configure outputPortInfo
    for(size_t i=0; i < ports;i++)
    {
        blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                              std::vector<int>{_joint_number},
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }

}

bool XBotBlock_MechanicalLimits::getMechanicalLimits(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info)
{
    // set all ouput of the list
    for(size_t i=0; i < _mechanical_limits_list.size();i++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i + _start_port);
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_mechanical_limits_options.count(_mechanical_limits_list[i]) > 0)
        {
            // save into auxiliary vector the mechanical limits information
            Eigen::VectorXd aux_vector;
            switch(_mechanical_limits_options.at(_mechanical_limits_list[i]))
            {
                case q_min: {
                                Eigen::VectorXd qmin,qmax;
                                _xbi->getJointLimits(qmin,qmax);
                                aux_vector = qmin;
                            }break;
                        
                case q_max: {
                                Eigen::VectorXd qmin,qmax;
                                _xbi->getJointLimits(qmin,qmax);
                                aux_vector = qmax;
                            }break;
                            
                case qdot_max:  {
                                    _xbi->getVelocityLimits(aux_vector);
                                }break;
                case tau_max:   {
                                    _xbi->getEffortLimits(aux_vector);
                                break;
                                }
                
            }
            
            // check the auxiliary vector size with output signal size
            if(aux_vector.size() != output->getWidth())
            {
                error_info = "Different dimension of output port: " + std::to_string(output->getWidth()) +" and mechanical limits vector: " + std::to_string(aux_vector.size());
                return false;
            }
            
            // set the signal output
            for (size_t  k=0; k < output->getWidth(); ++k)
            {
                output->set(k, aux_vector[k]);
            }
        }
        else
        {
            error_info = "Found unrecognized mechanical limits option from parameters selected";
            return false;
        }
    }
    return true;
}

// NOTE: only initialize fuction was developed for the mechanical limits, the output phase is not needed.

bool XBotBlock_MechanicalLimits::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    
    if(!getMechanicalLimits(blockInfo,error_info))
    {
        bfError << "Mechanical limits readings failed, reason: " << error_info;
        return false;
    }
    
    return true;
}
