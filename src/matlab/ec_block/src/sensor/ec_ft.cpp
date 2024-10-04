#include "sensor/ec_ft.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;

Ft::Ft(std::vector<std::string> ft_list,size_t start_port):
_ft_list(ft_list),
_start_port(start_port)
{    
   std::string error_info="";
   EcBlockUtils::retrieve_ft_info(_ft_id,error_info);
   
   _ft_number= _ft_id.size();
   if(_ft_number == 0)
   {
       _ft_number = 1;
   }
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool Ft::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    int i=0;
    for(const auto& ft_s:_ft_list)
    {
        if(_ft_option.count(ft_s) > 0)
        {
            std::vector<int> port_size;
            switch(_ft_option.at(ft_s))
            {

                case ft_id:{
                                port_size.push_back(_ft_number);
                            }break;
                case F:  {
                                port_size.push_back(3);
                                port_size.push_back(_ft_number);
                            }break;
                            
                case T: {
                            port_size.push_back(3);
                            port_size.push_back(_ft_number);
                        }break;
            }

            blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                                  port_size,
                                                  blockfactory::core::Port::DataType::DOUBLE};
            outputPortInfo.push_back(output);
            i++;
        }
        else
        {
            bfError << "Found unrecognized ft option from parameters selected";
            return false;
        }
    }
        
    return true;
}



bool Ft::getFt(const blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map,std::string &error_info)
{
    if(ft_status_map.empty())
    {
        error_info = "Got an empty ft status map";
        return false;
    }
    
    int port=0;
    for(const auto& ft:_ft_list)
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
        if(_ft_option.count(ft) > 0)
        {
            for(int ft_id_index=0;ft_id_index<_ft_number;ft_id_index++)
            {
                auto ft_id_read = _ft_id[ft_id_index];
                if(ft_status_map.count(ft_id_read) == 0)
                {
                    error_info = "Ft id: " + std::to_string(ft_id_read) + " not found in fts map status, please check ec conf file";
                    return false;
                }
                    
                auto ft_status_id = ft_status_map[ft_id_read];
                switch(_ft_option.at(ft))
                {
                    case ft_id:    {
                                        output->set(ft_id_index, ft_id_read);
                                    } break;
                    case F: {
                                output->set(3*ft_id_index,   std::get<0>(ft_status_id));
                                output->set(3*ft_id_index+1, std::get<1>(ft_status_id));
                                output->set(3*ft_id_index+2, std::get<2>(ft_status_id));
                            }break;
                    case T: {
                                output->set(3*ft_id_index,   std::get<3>(ft_status_id));
                                output->set(3*ft_id_index+1, std::get<4>(ft_status_id));
                                output->set(3*ft_id_index+2, std::get<5>(ft_status_id));
                                }break;              
                }
            }
        }
        else
        {
            bfError << "Found unrecognized ft components option from parameters selected";
            return false;
        }
    }

    return true;
}


bool Ft::initialize(blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map)
{
    std::string error_info="";
    if(!getFt(blockInfo,ft_status_map,error_info))
    {
        bfError << "Ft reading failed, reason: " << error_info;
        return false;
    }
    return true;
}

bool Ft::output(const blockfactory::core::BlockInformation* blockInfo,FtStatusMap ft_status_map)
{
    std::string error_info="";
    if(!getFt(blockInfo,ft_status_map,error_info))
    {
        bfError << "Ft reading failed, reason: " << error_info;
        return false;
    }
    return true;
}
