#include "XBotBlock/Managers/FloatingBase.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace XBB_FloatingBase;

using Matrix4dSimulink = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
using Matrix3dSimulink = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;


XBotBlock_FloatingBase::XBotBlock_FloatingBase(XBot::ModelInterface::Ptr model,
                                               std::vector<std::string> get_floating_base_list,
                                               std::string set_floating_base_component,
                                               size_t in_start_port,
                                               size_t out_start_port) :
_model(model),
_get_floating_base_list(get_floating_base_list),
_set_floating_base_component(set_floating_base_component),
_in_start_port(in_start_port),
_out_start_port(out_start_port)
{
    // create class and save the XBotInterface ptr, list and start port (input and output)
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_FloatingBase::configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                                                   blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    if(_set_floating_base_component !="Off")
    {
        std::vector<int> port_size,extra_port_size;
        if(_floating_base_option.count(_set_floating_base_component) > 0)
        {
            switch(_floating_base_option.at(_set_floating_base_component))
            {
                case Pose:  {
                                port_size.push_back(4);
                                port_size.push_back(4);
                            }break;          
                case State: {
                                port_size.push_back(4);
                                port_size.push_back(4);
                                extra_port_size.push_back(6);
                            }break;        
                case Orientation:   {
                                        port_size.push_back(3);
                                        port_size.push_back(3);
                                    }break; 
                case Twist: {
                                port_size.push_back(6);
                            }break;
                case AngularVelocity:   {
                                            port_size.push_back(3);
                                        }break;      
            }
            
        }
        else
        {
            bfError << "Found unrecognized pose components option from parameters selected";
            return false;
        }
    
        blockfactory::core::Port::Info input{/*portIndex=*/_in_start_port,
                                            port_size,
                                            blockfactory::core::Port::DataType::DOUBLE};
        inputPortInfo.push_back(input);
        
        
        if(!extra_port_size.empty())
        {
            blockfactory::core::Port::Info extra_input{/*portIndex=*/_in_start_port + 1,
                                                    extra_port_size,
                                                    blockfactory::core::Port::DataType::DOUBLE};
            inputPortInfo.push_back(extra_input);
        }
    }
    
    for(size_t i=0; i < _get_floating_base_list.size();i++)
    {
        std::vector<int> port_size;
        if(_floating_base_option.count(_get_floating_base_list[i]) > 0)
        {
            switch(_floating_base_option.at(_get_floating_base_list[i]))
            {
                case Pose:  {
                                port_size.push_back(4);
                                port_size.push_back(4);
                            }break;
                        
                case Twist: {
                                port_size.push_back(6);
                            }break;
                            
                default:    {
                                bfError << "Cannot get flaoting base on: " << _get_floating_base_list[i];
                                return false;
                            }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized pose components option from parameters selected";
            return false;
        }
        
        
        blockfactory::core::Port::Info ouput{/*portIndex=*/i + _out_start_port,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(ouput);
    }
    
    return true;
}

bool XBotBlock_FloatingBase::getFloatingBase(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
       // set all ouput of the list
        for(size_t i=0; i < _get_floating_base_list.size();i++)
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output_signal= blockInfo->getOutputPortSignal(/*index=*/i +_out_start_port);
            // Check the signal validity
            if (!output_signal) {
                bfError << "Signal not valid";
                return false;
            }
            // verify if the element of the list exists like option
            if(_floating_base_option.count(_get_floating_base_list[i]) > 0)
            {
                switch(_floating_base_option.at(_get_floating_base_list[i]))
                {
                    case Pose:  {
                                    if(!_model->getFloatingBasePose(_get_pose_frame))
                                    {
                                        bfError << "Model is not floating base";
                                        return false;
                                    }    
                                    // Allocate objects for row-major -> col-major conversion
                                    Eigen::Map<Matrix4dSimulink> HomogTransfolMajor(output_signal->getBuffer<double>(),4,4);
                                    
                                    HomogTransfolMajor=_get_pose_frame.matrix();   
                                    
                                }break;
                    case Twist: {
                                    if(!_model->getFloatingBaseTwist(_get_twist))
                                    {
                                        bfError << "Model is not floating base";
                                        return false;
                                    } 
                                    
                                    for (size_t  k=0; k < _get_twist.size(); ++k)
                                    {
                                        output_signal->set(k, _get_twist[k]);
                                    }

                                }break;
                    default:    {
                                    bfError << "Cannot get flaoting base on: " << _get_floating_base_list[i];
                                    return false;
                                }break;
                }
            }
        }
    }

    return true;
}

bool XBotBlock_FloatingBase::setFloatingBase(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
        // verify if the element of the list exists like option
        if(_floating_base_option.count(_set_floating_base_component) > 0)
        {
            // Get general signal input
            blockfactory::core::InputSignalPtr input_signal = blockInfo->getInputPortSignal(/*index=*/_in_start_port);
            
            // Check the signal validity
            if (!input_signal) {
                bfError << "general input signal not valid";
                return false;
            }
            
            switch(_floating_base_option.at(_set_floating_base_component))
            {
                case Pose:  {
                                const double* HT_buffer = input_signal->getBuffer<double>();
                                if (!HT_buffer) {
                                    bfError << "Failed to read the homogeneous transformation matrix from input port.";
                                    return false;
                                }
                                
                                _set_pose_frame.matrix()=Matrix4dSimulink(HT_buffer);

                                if(!_model->setFloatingBasePose(_set_pose_frame))
                                {
                                    bfError << "Error to set floating base pose";
                                    bfError << "Model could not be floating base or problem with the input pose frame";
                                    return false;
                                }    
                            }break;
                case State:  {
                                // Get general signal input
                                blockfactory::core::InputSignalPtr twist_signal = blockInfo->getInputPortSignal(/*index=*/_in_start_port+1);
                                
                                // Check the signal validity
                                if (!twist_signal) {
                                    bfError << "Twist signal signal not valid";
                                    return false;
                                }
                        
                    
                                const double* HT_buffer = input_signal->getBuffer<double>();
                                if (!HT_buffer) {
                                    bfError << "Failed to read the homogeneous transformation matrix from input port.";
                                    return false;
                                }
                                
                                _set_pose_frame.matrix()=Matrix4dSimulink(HT_buffer);
                                
                                if(twist_signal->getWidth()!=6)
                                {
                                    bfError << "Twist signal must a vector with size equal to 6";
                                    return false;
                                }
                                
                                for (size_t  k=0; k < _set_twist.size(); ++k)
                                {
                                    _set_twist[k]=twist_signal->get<double>(k);
                                }

                                if(!_model->setFloatingBaseState(_set_pose_frame,_set_twist))
                                {
                                    bfError << "Error to set floating base state";
                                    bfError << "Model could not be floating base or problem with the input pose frame / twist ";
                                    return false;
                                }      
                            }break;
                case Orientation:   {
                                        const double* orientation_buffer = input_signal->getBuffer<double>();
                                        if (!orientation_buffer) {
                                            bfError << "Failed to read the rotation matrix from input port.";
                                            return false;
                                        }
                                        
                                        _orientation.matrix()=Matrix3dSimulink(orientation_buffer);

                                        if(!_model->setFloatingBaseOrientation(_orientation))
                                        {
                                            bfError << "Error to set floating base orientation";
                                            bfError << "Model could not be floating base or problem orientation matrix";
                                            return false;
                                        }    
                                    }break;
                case Twist: {
                                if(input_signal->getWidth()!=6)
                                {
                                    bfError << "Twist signal must a vector with size equal to 6";
                                    return false;
                                } 
                                for (size_t  k=0; k < _set_twist.size(); ++k)
                                {
                                    _set_twist[k]=input_signal->get<double>(k);
                                }
                                if(!_model->setFloatingBaseTwist(_set_twist))
                                {
                                    bfError << "Error to set floating base twist";
                                    bfError << "Model could not be floating base or problem with the input twist";
                                    return false;
                                } 
                            }break;
                case AngularVelocity:   {
                                            if(input_signal->getWidth()!=3)
                                            {
                                                bfError << "Angular velocity signal must a vector with size equal to 3";
                                                return false;
                                            }
                                            
                                            for (size_t  k=0; k < _angular_velocity.size(); ++k)
                                            {
                                                _angular_velocity[k]=input_signal->get<double>(k);
                                            }
                                            if(!_model->setFloatingBaseAngularVelocity(_angular_velocity))
                                            {
                                                bfError << "error to set floating angular velocity";
                                                bfError << "Model could not be floating base or problem with the input angular velocity";
                                                return false;
                                            } 
                                        }break;
            }
        }
    }

    return true;
}

bool XBotBlock_FloatingBase::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    if(_set_floating_base_component != "Off")
    {
        if(!setFloatingBase(blockInfo))
        {
            return false;
        }
        // model update
        _model->update();
    }

    if(!getFloatingBase(blockInfo))
    {
        return false;
    }
    
    return true;
}


bool XBotBlock_FloatingBase::output(const blockfactory::core::BlockInformation* blockInfo)
{

    if(_set_floating_base_component != "Off")
    {
        if(!setFloatingBase(blockInfo))
        {
            return false;
        }
        // model update
        _model->update();
    }
    
    if(!getFloatingBase(blockInfo))
    {
        return false;
    }
    
    return true;
}
