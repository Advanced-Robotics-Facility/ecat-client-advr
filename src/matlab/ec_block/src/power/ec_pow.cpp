#include "power/ec_pow.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;

Pow::Pow(std::vector<std::string> pow_list,size_t start_port):
_pow_list(pow_list),
_start_port(start_port)
{    
   std::string error_info="";
   EcBlockUtils::retrieve_pow_info(_pow_id,error_info);
   
   _pow_number= _pow_id.size();
   if(_pow_number == 0)
   {
       _pow_number = 1;
   }
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool Pow::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    int i=0;
    for(const auto& pow_s:_pow_list)
    {
        if(_pow_option.count(pow_s) > 0)
        {
            std::vector<int> port_size;
            
            port_size.push_back(1);
            port_size.push_back(_pow_number);
           

            blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                                  port_size,
                                                  blockfactory::core::Port::DataType::DOUBLE};
            outputPortInfo.push_back(output);
            i++;
        }
        else
        {
            bfError << "Found unrecognized pow option from parameters selected";
            return false;
        }
    }
        
    return true;
}



bool Pow::getPow(const blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map,std::string &error_info)
{
    if(pow_status_map.empty())
    {
        error_info = "Got an empty pow status map";
        return false;
    }
    
    int port=0;
    for(const auto& pow:_pow_list)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/port + _start_port);
        port++;
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_pow_option.count(pow) > 0)
        {
            for(int pow_id_index=0;pow_id_index<_pow_number;pow_id_index++)
            {
                auto pow_id_read = _pow_id[pow_id_index];
                if(pow_status_map.count(pow_id_read) == 0)
                {
                    error_info = "Pow id: " + std::to_string(pow_id_read) + " not found in pows map status, please check ec conf file";
                    return false;
                }
                    
                auto pow_status_id = pow_status_map[pow_id_read];
                switch(_pow_option.at(pow))
                {
                    case pow_id:    {
                                        output->set(pow_id_index, pow_id_read);
                                    } break;
                    case v_batt:  {
                                    output->set(pow_id_index, std::get<0>(pow_status_id));
                                }break;
                    case v_load:  {
                                    output->set(pow_id_index, std::get<1>(pow_status_id));
                                }break;
                    case i_load:  {
                                    output->set(pow_id_index, std::get<2>(pow_status_id));
                                }break;
                                
                    case temp_pcb:  {
                                    output->set(pow_id_index, std::get<3>(pow_status_id));
                                }break;
                    case temp_heatsink:  {
                                    output->set(pow_id_index, std::get<4>(pow_status_id));
                                }break;
                    case temp_batt:  {
                                    output->set(pow_id_index, std::get<5>(pow_status_id));
                                }break;
                }
            }
        }
        else
        {
            bfError << "Found unrecognized pow components option from parameters selected";
            return false;
        }
    }

    return true;
}


bool Pow::initialize(blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map)
{
    std::string error_info="";
    if(!getPow(blockInfo,pow_status_map,error_info))
    {
        bfError << "Pow reading failed, reason: " << error_info;
        return false;
    }
    return true;
}

bool Pow::output(const blockfactory::core::BlockInformation* blockInfo,PwrStatusMap pow_status_map)
{
    std::string error_info="";
    if(!getPow(blockInfo,pow_status_map,error_info))
    {
        bfError << "Pow reading failed, reason: " << error_info;
        return false;
    }
    return true;
}
