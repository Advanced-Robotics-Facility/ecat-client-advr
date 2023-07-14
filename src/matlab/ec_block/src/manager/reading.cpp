#include "manager/reading.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;


Reading::Reading(std::vector<std::string> readings_list,
                 size_t start_port):
_readings_list(readings_list),
_start_port(start_port)
{    
   std::string error_info="";
   std::vector<float> gains;
   EcBlockUtils::retrieve_ec_info(_joint_number,_q_id,_ctrl_mode,gains,_q_home,_q_trj,error_info);
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

bool Reading::getReadings(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref,std::string &error_info)
{
    if(motors_status_map.empty())
    {
        error_info = "Got an empty motor status map";
        return false;
    }
    
    if(motors_ref.empty())
    {
        error_info = "Got an empty motor references structure for reading";
        return false;
    }

    // set all ouput of the list
    for(size_t port=0; port < _readings_list.size();port++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/port + _start_port);
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_readings_options.count(_readings_list[port]) > 0)
        {
            // save into auxiliary vector the readings information
            Eigen::VectorXd aux_vector;
            aux_vector.resize(_q_id.size());
            
            switch(_readings_options.at(_readings_list[port]))
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
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    MR mot_ref = motors_ref[i];
                                    aux_vector[i]=std::get<5>(mot_ref) ;
                                }
                            }break;
                case gainD:{
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    MR mot_ref = motors_ref[i];
                                    if(_ctrl_mode == 0xD4)
                                    {
                                        std::get<6>(mot_ref) = aux_vector[i];
                                    }
                                    else
                                    {
                                        std::get<7>(mot_ref) = aux_vector[i];
                                    }
                                }
                            }break;
                case qJ_ref:{
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    MR mot_ref = motors_ref[i];
                                    aux_vector[i]=std::get<2>(mot_ref) ;
                                }
                            }break;
                case qJdot_ref:{
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    MR mot_ref = motors_ref[i];
                                    aux_vector[i]=std::get<3>(mot_ref) ;
                                }
                            }break;
                case tau_ref:{
                                for(int i=0;i<aux_vector.size();i++)
                                {
                                    MR mot_ref = motors_ref[i];
                                    aux_vector[i]=std::get<4>(mot_ref) ;
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

bool Reading::initialize(blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref)
{
    std::string error_info="";
    if(!getReadings(blockInfo,motors_status_map,motors_ref,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }

    return true;
}


bool Reading::output(const blockfactory::core::BlockInformation* blockInfo,MotorStatusMap motors_status_map,std::vector<MR> motors_ref)
{
    std::string error_info="";
    if(!getReadings(blockInfo,motors_status_map,motors_ref,error_info))
    {
        bfError << "Joint readings failed, reason: " << error_info;
        return false;
    }
    return true;    
}
