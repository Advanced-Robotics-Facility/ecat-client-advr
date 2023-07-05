#include "ModelName_rt_plugin.h"

#include "ModelName.h"
#include <XBotBlock/Common/Utils.h>

bool ModelName_Plugin::on_initialize()
{

    // Avoid ROS communication
    CommonUtils::setXBotCorePresence(true);
    std::string error_info="";
    
    // Insert the robot and model interface
    if(!CommonUtils::setExternalRobot("DefaultRobot",_robot,error_info))
    {
       jerror("could set the robot: '{}'\n", error_info); 
       return false;
    }
    
    XBot::ConfigOptions robot_config=_robot->getConfigOptions(); 
    _rt_model=XBot::ModelInterface::getModel(robot_config);
    
    if(_rt_model == nullptr)
    {
       jerror("got a null model \n"); 
       return false;
    }
            
    if(!CommonUtils::setExternalModel("DefaultModel",_rt_model,error_info))    
    {
       jerror("could set the model: '{}'\n", error_info); 
       return false;
    }
    
    
    // Default position mode.
    _robot->setControlMode(ControlMode::Position());
    
    // we must explicitly set the control mode for our robot
    // in this case, we will only send positions
    // Note: activate it using the R-HAL 
    //set_control_mode();

    // plugin flag init
    _plugin_started=false;
    
    return true;
}

void ModelName_Plugin::set_control_mode()
{
    std::map<std::string, ControlMode> ctrl_map;
    
    for(auto j : _robot->getEnabledJointNames())
    {
        ControlMode ctrl;
        if(!getParam("/hal/joint/control_mode/" + j, ctrl))
        {
            jwarn("could not find control mode for joint '{}', setting to idle \n", j);
        }
        
        ctrl_map[j] = ctrl;
    }
    _robot->setControlMode(ctrl_map);
}

void ModelName_Plugin::starting()
{
    _plugin_started=false;
    
    ModelName_initialize();
    
    _plugin_started=true;

    start_completed();
}

void ModelName_Plugin::run()
{
   ModelName_step();
}

void ModelName_Plugin::on_close()
{
    if(_plugin_started)
    {
    	ModelName_terminate();
    }
}

XBOT2_REGISTER_PLUGIN(ModelName_Plugin, model_name_plugin)
