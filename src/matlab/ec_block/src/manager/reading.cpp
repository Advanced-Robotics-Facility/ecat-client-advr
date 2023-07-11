#include "manager/reading.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;


Reading::Reading(EcIface::Ptr robot,
                 std::vector<std::string> readings_list,
                 size_t start_port):
_robot(robot),
_readings_list(readings_list),
_start_port(start_port)
{    
   std::string error_info="";
   _joint_number = EcBlockUtils::retrive_joint_numb(error_info);
   _q_id = EcBlockUtils::retrive_joint_id();
   _ctrl_mode=EcBlockUtils::retrive_ctrl_mode();
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

bool Reading::getReadings(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::string &error_info)
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
            aux_vector.resize(_q_id.size());
            
            switch(_readings_options.at(_readings_list[i]))
            {
                case q_ID: {
                            for(int i=0;i<aux_vector.size();i++)
                            {
                                aux_vector[i]=_q_id[i];
                            }
                        }break;
                case qJ: {
                            for(int i=0;i<aux_vector.size();i++)
                            {
                                aux_vector[i]=std::get<0>(motors_status_map[_q_id[i]]);
                            }
                        }break;
                        
                case qM: {
                            for(int i=0;i<aux_vector.size();i++)
                            {
                                aux_vector[i]=std::get<1>(motors_status_map[_q_id[i]]);
                            }
                        }break;
                        
                case qJdot: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<2>(motors_status_map[_q_id[i]]);
                                }
                            }break;
                            
                case qMdot: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<3>(motors_status_map[_q_id[i]]);
                                }
                            }break;
                case tau:   {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<4>(motors_status_map[_q_id[i]]);
                                }
                            }break; 
                case qTemp: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<5>(motors_status_map[_q_id[i]]);
                                }
                            }break;
                case bTemp: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<5>(motors_status_map[_q_id[i]]);
                                }
                            }break;
                case gainP:{
                                auto gains = EcBlockUtils::retrieve_joint_gains();
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=gains[0];
                                }
                            }break;
                case gainD:{
                                auto gains = EcBlockUtils::retrieve_joint_gains();
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    if(_ctrl_mode == 0xD4)
                                    {
                                        aux_vector[i]=gains[1];
                                    }
                                    else
                                    {
                                        aux_vector[i]=gains[2];
                                    }
                                }
                            }break;
                case fault: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<7>(motors_status_map[_q_id[i]]);
                                }
                            }break;
                case cmd_aux_sts: {
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    aux_vector[i]=std::get<11>(motors_status_map[_q_id[i]]);
                                }
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

bool Reading::initialize(blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map)
{
    std::string error_info="";
    if(!getReadings(blockInfo,motors_status_map,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }

    return true;
}


bool Reading::output(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map)
{
    std::string error_info="";
    if(!getReadings(blockInfo,motors_status_map,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }
    return true;    
}
