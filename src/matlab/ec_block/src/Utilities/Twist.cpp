#include "XBotBlock/Utilities/Twist.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace XBB_Twist;


XBotBlock_Twist::XBotBlock_Twist(XBot::ModelInterface::Ptr model,
                                 std::string link_name,
                                 std::string base_link_name,
                                 std::vector<std::string> twist_list,
                                 size_t start_port):
_model(model),
_link_name(link_name),
_base_link_name(base_link_name),
_twist_list(twist_list),
_start_port(start_port)
{
    // create class and save the ModelInterface ptr, list and start port (input and output)
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
void XBotBlock_Twist::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    size_t ports = _twist_list.size();
    
    // configure outputPortInfo
    for(size_t i=0; i < ports;i++)
    {
        blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                              std::vector<int>{6},
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }
}

bool XBotBlock_Twist::getTwist(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info)
{
    // set all ouput of the list
    for(size_t i=0; i < _twist_list.size();i++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i + _start_port);
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_twist_option.count(_twist_list[i]) > 0)
        {
            // save into auxiliary vector the readings information
            Eigen::Vector6d aux_vector;
            switch(_twist_option.at(_twist_list[i]))
            {
                case v_tw:  {
                                if(!_model->getVelocityTwist(_link_name,aux_vector))
                                {
                                    error_info ="Invalid link name";
                                    return false;
                                }
                            }break;
                case a_tw:  {
                                if(!_model->getAccelerationTwist(_link_name,aux_vector))
                                {
                                    error_info ="Invalid link name";
                                    return false;
                                }
                            }break;
                        
                case v_tw_rel:  {
                                    if(!_model->getRelativeVelocityTwist(_link_name,_base_link_name,aux_vector))
                                    {                                
                                        error_info ="Invalid link or base link name";
                                        return false;
                                    }
                                }break;
                        
                case a_tw_rel:  {
                                    if(!_model->getRelativeAccelerationTwist(_link_name,_base_link_name,aux_vector))
                                    {                                
                                        error_info ="Invalid link or base link name";
                                        return false;
                                    }
                                }break;
            }
            
            // check the auxiliary vector size with output signal size
            if(aux_vector.size() != output->getWidth())
            {
                error_info = "Different dimension of output port: " + std::to_string(output->getWidth()) +" and twist vector: " + std::to_string(aux_vector.size());
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
            error_info = "Found unrecognized twist option from parameters selected";
            return false;
        }
    }
    return true;
}

bool XBotBlock_Twist::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    if(!getTwist(blockInfo,error_info))
    {
        bfError << "Twist getting failed, reason: " << error_info;
        return false;
    }

    return true;
}


bool XBotBlock_Twist::output(const blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    if(!getTwist(blockInfo,error_info))
    {
        bfError << "Twist getting failed, reason: " << error_info;
        return false;
    }
    return true;    
}
