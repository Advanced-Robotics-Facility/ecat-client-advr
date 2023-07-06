#include "manager/reading.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;


Reading::Reading(XBot::XBotInterface::Ptr xbi,
                 std::vector<std::string> readings_list,
                 size_t start_port):
_xbi(xbi),
_readings_list(readings_list),
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
void Reading::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    size_t ports = _readings_list.size();
    
    // configure outputPortInfo
    for(size_t i=0; i < ports;i++)
    {
        blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                              std::vector<int>{_joint_number},
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }
}

bool Reading::getReadings(const blockfactory::core::BlockInformation* blockInfo,std::string &error_info)
{
    // set all ouput of the list
    for(size_t i=0; i < _readings_list.size();i++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i + _start_port);
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_readings_options.count(_readings_list[i]) > 0)
        {
            // save into auxiliary vector the readings information
            Eigen::VectorXd aux_vector;
            switch(_readings_options.at(_readings_list[i]))
            {
                case q_ID: {
                            std::vector<int> q_id=_xbi->getEnabledJointId();
                            aux_vector.resize(q_id.size());
                            for(int i=0;i<aux_vector.size();i++)
                            {
                                aux_vector[i]=q_id[i];
                            }
                        }break;
                case qJ: {
                            _xbi->getJointPosition(aux_vector);
                        }break;
                        
                case qM: {
                            _xbi->getMotorPosition(aux_vector);
                        }break;
                        
                case qJdot:  {
                                _xbi->getJointVelocity(aux_vector);
                            }break;
                            
                case qMdot:  {
                                _xbi->getMotorVelocity(aux_vector);
                            }break;
                case qJddot:    {
                                    _xbi->getJointAcceleration(aux_vector);
                                }break;
                case tau:   {
                                _xbi->getJointEffort(aux_vector);
                            }break;
                case qTemp: {
                                _xbi->getTemperature(aux_vector);
                            }break;
                case K: {
                            _xbi->getStiffness(aux_vector);
                        }break;
                case D: {
                            _xbi->getDamping(aux_vector);
                        }break;
                case qJ_ref: {
                                _xbi->getPositionReference(aux_vector);
                            }break;
                        
                case qJdot_ref:  {
                                    _xbi->getVelocityReference(aux_vector);
                                }break;
                            
                case tau_ref:   {
                                _xbi->getEffortReference(aux_vector);
                            }break;
                
            }
            
            // check the auxiliary vector size with output signal size
            // NOTE this allow to avoid the checks (bool return) on getting function
            if(aux_vector.size() != output->getWidth())
            {
                error_info = "Different dimension of output port: " + std::to_string(output->getWidth()) +" and joint readings vector: " + std::to_string(aux_vector.size());
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
            error_info = "Found unrecognized joint reading option from parameters selected";
            return false;
        }
    }
    return true;
}

bool Reading::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    if(!getReadings(blockInfo,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }

    return true;
}


bool Reading::output(const blockfactory::core::BlockInformation* blockInfo)
{
    std::string error_info="";
    if(!getReadings(blockInfo,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }
    return true;    
}
