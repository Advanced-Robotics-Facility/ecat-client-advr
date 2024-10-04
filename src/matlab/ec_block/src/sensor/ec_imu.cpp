#include "sensor/ec_imu.h"

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace EcBlock;

Imu::Imu(std::vector<std::string> imu_list,size_t start_port):
_imu_list(imu_list),
_start_port(start_port)
{    
   std::string error_info="";
   EcBlockUtils::retrieve_imu_info(_imu_id,error_info);
   
   _imu_number= _imu_id.size();
   if(_imu_number == 0)
   {
       _imu_number = 1;
   }
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool Imu::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    int i=0;
    for(const auto& imu_s:_imu_list)
    {
        if(_imu_option.count(imu_s) > 0)
        {
            std::vector<int> port_size;
            switch(_imu_option.at(imu_s))
            {

                case imu_id:{
                                port_size.push_back(_imu_number);
                            }break;
                case quat:  {
                                port_size.push_back(4);
                                port_size.push_back(_imu_number);
                            }break;
                            
                case a: {
                            port_size.push_back(3);
                            port_size.push_back(_imu_number);
                        }break;
                case omega: {
                                port_size.push_back(3);
                                port_size.push_back(_imu_number);
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
            bfError << "Found unrecognized imu option from parameters selected";
            return false;
        }
    }
        
    return true;
}



bool Imu::getImu(const blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map,std::string &error_info)
{
    if(imu_status_map.empty())
    {
        error_info = "Got an empty imu status map";
        return false;
    }
    
    int port=0;
    for(const auto& imu:_imu_list)
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
        if(_imu_option.count(imu) > 0)
        {
            for(int imu_id_index=0;imu_id_index<_imu_number;imu_id_index++)
            {
                auto imu_id_read = _imu_id[imu_id_index];
                if(imu_status_map.count(imu_id_read) == 0)
                {
                    error_info = "Imu id: " + std::to_string(imu_id_read) + " not found in imus map status, please check ec conf file";
                    return false;
                }
                    
                auto imu_status_id = imu_status_map[imu_id_read];
                switch(_imu_option.at(imu))
                {
                    case imu_id:    {
                                        output->set(imu_id_index, imu_id_read);
                                    } break;
                    case quat:  {
                                    output->set(4*imu_id_index,   std::get<9>(imu_status_id));
                                    output->set(4*imu_id_index+1, std::get<6>(imu_status_id));
                                    output->set(4*imu_id_index+2, std::get<7>(imu_status_id));
                                    output->set(4*imu_id_index+3, std::get<8>(imu_status_id));
                                }break;
                    case a: {
                                output->set(3*imu_id_index,   std::get<0>(imu_status_id));
                                output->set(3*imu_id_index+1, std::get<1>(imu_status_id));
                                output->set(3*imu_id_index+2, std::get<2>(imu_status_id));
                            }break;
                    case omega: {
                                output->set(3*imu_id_index,   std::get<3>(imu_status_id));
                                output->set(3*imu_id_index+1, std::get<4>(imu_status_id));
                                output->set(3*imu_id_index+2, std::get<5>(imu_status_id));
                                }break;              
                }
            }
        }
        else
        {
            bfError << "Found unrecognized imu components option from parameters selected";
            return false;
        }
    }

    return true;
}


bool Imu::initialize(blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map)
{
    std::string error_info="";
    if(!getImu(blockInfo,imu_status_map,error_info))
    {
        bfError << "Imu reading failed, reason: " << error_info;
        return false;
    }
    return true;
}

bool Imu::output(const blockfactory::core::BlockInformation* blockInfo,ImuStatusMap imu_status_map)
{
    std::string error_info="";
    if(!getImu(blockInfo,imu_status_map,error_info))
    {
        bfError << "Imu reading failed, reason: " << error_info;
        return false;
    }
    return true;
}
