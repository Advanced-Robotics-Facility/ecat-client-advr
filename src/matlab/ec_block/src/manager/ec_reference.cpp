#include "manager/ec_reference.h"
#include "ec_block_utils.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;

Reference::Reference(std::vector<std::string> references_list,
                     size_t in_start_port):
_references_list(references_list),
_in_start_port(in_start_port)
{
    std::string error_info="";
    std::vector<float> gains;
    std::vector<double> q_home,q_trj;
    EcBlockUtils::retrieve_motor_info(_q_id,_ctrl_mode,gains,q_home,q_trj,error_info);
   
    _joint_number= _q_id.size();
   if(_joint_number == 0)
   {
       _joint_number = 1;
   }
}


// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
void Reference::configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo)
{
    size_t ports = _references_list.size();
    
    // configure inputPortInfo
    for(size_t i=0; i < ports;i++)
    {
        blockfactory::core::Port::Info input{/*portIndex=*/i + _in_start_port,
                                              std::vector<int>{_joint_number},
                                              blockfactory::core::Port::DataType::DOUBLE};
        inputPortInfo.push_back(input);
    }
}


bool Reference::setReferences(const blockfactory::core::BlockInformation* blockInfo,MotorReferenceMap &motors_ref,std::string &error_info)
{
    if(motors_ref.empty())
    {
        error_info = "Got an empty motor references structure";
        return false;
    }
    
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
        int input_width = static_cast<int>(input->getWidth());
        if(input_width != _joint_number)
        {
            error_info = "Different dimension of input port: " + std::to_string(input->getWidth()) +" and joint number: " + std::to_string(_joint_number);
            return false;
        }
        
        // use an auxiliary vectory to save the input signal values
        Eigen::VectorXd aux_vector;
        aux_vector.resize(_joint_number);
        for (int  k=0; k < aux_vector.size(); ++k)
        {
            aux_vector[k]=input->get<double>(k);
        }
        // verify if the element of the list exists like option
        if(_references_options.count(_references_list[i]) > 0)
        {
            size_t j=0;
            for ( auto &[bId,motor_ref] : motors_ref ) {
                // set aux_vector like reference 
                switch(_references_options.at(_references_list[i]))
                {
                    case qJ_ref: {   
                                    std::get<1>(motor_ref) = aux_vector[j];
                                }break;
                    
                                
                    case qJdot_ref: {
                                        std::get<2>(motor_ref) = aux_vector[j];
                                    }break;   
                    
                
                    case tau_ref:   {
                                        std::get<3>(motor_ref) = aux_vector[j];
                                    }break;

                    case gainP_ref: {
                                    std::get<4>(motor_ref) = aux_vector[j];
                                }break;
                    case gainD_ref: {
                                    if(_ctrl_mode == 0xD4)
                                    {
                                        std::get<5>(motor_ref) = aux_vector[j];
                                    }
                                    else
                                    {
                                        std::get<6>(motor_ref) = aux_vector[j];
                                    }
                                }break;
                }
                j++;
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

bool Reference::output(const blockfactory::core::BlockInformation* blockInfo,MotorReferenceMap &motors_ref)
{
    std::string error_info="";
    if(!setReferences(blockInfo,motors_ref,error_info))
    {
        bfError << "Joint references failed, reason: " << error_info;
        return false;
    }
    return true;    
}
