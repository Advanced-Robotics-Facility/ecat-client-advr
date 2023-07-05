#include "XBotBlock/Managers/SensorManager.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>


using namespace XBB_SensorManager;

XBotBlock_SensorManager::XBotBlock_SensorManager(XBot::XBotInterface::Ptr xbi,
                                                 std::vector<std::string> sensor_list,
                                                 size_t start_port):
_xbi(xbi),
_sensor_list(sensor_list),
_start_port(start_port)
{
    // create class and save the XBotInterface ptr, list and start port (input and output)
    // initialize joint_number to one
    
    //override joint_number
    if(_xbi != nullptr)
    {
        _imu_map = _xbi->getImu();
        _ft_map = _xbi->getForceTorque();
    }
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_SensorManager::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    for(size_t i=0; i < _sensor_list.size();i++)
    {
        std::vector<int> port_size;
        if(_sensor_options.count(_sensor_list[i]) > 0)
        {
            switch(_sensor_options.at(_sensor_list[i]))
            {
                case IMU:   {
                                size_t imu_numb = _imu_map.size();
                                if(imu_numb == 0)
                                {
                                    imu_numb = 1;
                                }
                                port_size.push_back(imu_numb);
                            }break;
                        
                case FT:    {
                                size_t ft_numb = _ft_map.size();
                                if(ft_numb == 0)
                                {
                                    ft_numb = 1;
                                }
                                port_size.push_back(ft_numb);
                            }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized sensor from parameters selected";
            return false;
        }
        
        blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }
    
    return true;
}


bool XBotBlock_SensorManager::initialize(blockfactory::core::BlockInformation* blockInfo)
{
       
    for(size_t i=0; i < _sensor_list.size();i++)
    {
        // get ouput signal
        blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i +_start_port);
        // Check the signal validity
        if (!output) {
            bfError << "Signal not valid";
            return false;
        }
        
        std::vector<int> id_sensor;
        if(_sensor_options.count(_sensor_list[i]) > 0)
        {
            switch(_sensor_options.at(_sensor_list[i]))
            {
                case IMU:   {
                                if(_imu_map.empty())
                                {
                                    id_sensor.push_back(-1); // NO FT
                                }
                                else
                                {
                                    for(auto& [imu_name,imu_ptr]:_imu_map)
                                    {
                                        id_sensor.push_back(imu_ptr->getSensorId());
                                    }
                                }
                            }break;
                        
                case FT:   {
                                if(_ft_map.empty())
                                {
                                    id_sensor.push_back(-1); // NO FT
                                }
                                else
                                {
                                    for(auto& [ft_name,ft_ptr]:_ft_map)
                                    {
                                        id_sensor.push_back(ft_ptr->getSensorId());
                                    } 
                                }
                            }break;
            }
            
            // check the auxiliary vector size with output signal size
            if(id_sensor.size() != output->getWidth())
            {
                bfError << "Different dimension of output port: " + std::to_string(output->getWidth()) +" and sensor id vector: " + std::to_string(id_sensor.size());
                return false;
            }
            
            // set the signal output
            for (size_t  k=0; k < output->getWidth(); ++k)
            {
                output->set(k, id_sensor[k]);
            }
            
        }
        else
        {
            bfError << "Found unrecognized sensor from parameters selected";
            return false;
        }
        
    }
    
    return true;
}
