#include <BlockFactory/Core/Block.h>
#include <BlockFactory/Core/BlockInformation.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/ImuSensor.h>
#include <XBotInterface/ForceTorqueSensor.h>

#include <memory>
#include <string>

namespace XBB_SensorManager {
    class XBotBlock_SensorManager;
}

class XBB_SensorManager::XBotBlock_SensorManager
{
private:
    
    XBot::XBotInterface::Ptr _xbi;
    
    std::vector<std::string> _sensor_list;
    std::map< std::string, XBot::ImuSensor::ConstPtr > _imu_map;
    std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > _ft_map;
    
    size_t _start_port;

    enum Sensor_Options
    {
        IMU,
        FT,
    };
    
    std::map<std::string, Sensor_Options> _sensor_options {
        { "IMU", IMU },
        { "FT", FT }
    };

public:
 
    static const std::string ClassName;
    
    
    XBotBlock_SensorManager(XBot::XBotInterface::Ptr xbi,
                               std::vector<std::string> sensor_list,
                               size_t start_port);
    ~XBotBlock_SensorManager(){};

    bool configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo);
    bool initialize(blockfactory::core::BlockInformation* blockInfo);
};

