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
    
    for(size_t i=0; i < _imu_list.size();i++)
    {
        if(_imu_option.count(_imu_list[i]) > 0)
        {
            std::vector<int> port_size;
            if(_imu_list[i]=="imu_id")
            {
                port_size.push_back(_imu_number);
            }
            else
            {
                for(size_t k=0; k < _imu_number;k++)
                {
                    std::cout << _imu_list[i] << std::endl;
                    switch(_imu_option.at(_imu_list[i]))
                    {

                        case imu_id:{}break;
                        case quat:  {
                                        port_size.push_back(4);
                                    }break;
                                    
                        case a: {
                                    port_size.push_back(3);
                                }break;
                        case omega: {
                                        port_size.push_back(3);
                                    }break;
                    }
                }
            }
            
            blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                                  port_size,
                                                  blockfactory::core::Port::DataType::DOUBLE};
            outputPortInfo.push_back(output);
            
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
    
    for(size_t port=0; port < _imu_list.size();port++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/port + _start_port);
        // Check the signal validity
        if (!output) {
            error_info = "Signal not valid";
            return false;
        }
        // verify if the element of the list exists like option
        if(_imu_option.count(_imu_list[port]) > 0)
        {
            if(_imu_list[port]=="imu_id")
            {
                for(int imu_id_index=0;imu_id_index<_imu_number;imu_id_index++)
                {
                    output->set(imu_id_index, _imu_id[imu_id_index]);
                }
            }
            else
            {
//                 for(int imu_id_index=0;imu_id_index<_imu_number;imu_id_index++)
//                 {
//                     auto imu_status_id = imu_status_map[imu_id_index];
//                     // save into auxiliary vector the IMU information
//                     Eigen::Vector3d aux_vector;
//                     switch(_imu_option.at(_imu_list[port]))
//                     {
//                         case imu_id:{}break;
//                         case quat:  {
//                                         output->set(imu_id_index, imu_status_id[9]);
//                                         output->set(imu_id_index+1, imu_status_id[6]);
//                                         output->set(imu_id_index+2, imu_status_id[7]);
//                                         output->set(imu_id_index+3, imu_status_id[8]);
//                                     }break;
//                         case a: {
//                                     output->set(imu_id_index, imu_status_id[0]);
//                                     output->set(imu_id_index+1, imu_status_id[1]);
//                                     output->set(imu_id_index+2, imu_status_id[2]);
//                                 }break;
//                         case omega: {
//                                     output->set(imu_id_index, imu_status_id[3]);
//                                     output->set(imu_id_index+1, imu_status_id[4]);
//                                     output->set(imu_id_index+2, imu_status_id[5]);
//                                     }break;              
//                     }
//                 }
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
