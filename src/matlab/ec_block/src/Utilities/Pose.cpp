#include "XBotBlock/Utilities/Pose.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>


using Matrix4dSimulink = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
using Matrix3dSimulink = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;

using namespace XBB_Pose;

XBotBlock_Pose::XBotBlock_Pose(XBot::ModelInterface::Ptr model,
                               std::string target_frame,
                               std::string source_frame,
                               std::string link_name,
                               std::vector<std::string> pose_list,
                               size_t in_start_port,
                               size_t out_start_port):
_model(model),
_target_frame(target_frame),
_source_frame(source_frame),
_link_name(link_name),
_pose_list(pose_list),
_in_start_port(in_start_port),
_out_start_port(out_start_port)
{
    // create class and save the ModelInterface ptr, list and start port (input and output)
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_Pose::configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                                           blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    for(size_t i=0; i < _pose_list.size();i++)
    {
        std::vector<int> port_size;
        if(_pose_option.count(_pose_list[i]) > 0)
        {
            switch(_pose_option.at(_pose_list[i]))
            {
                case Pose:  {
                                port_size.push_back(4);
                                port_size.push_back(4);
                            }break;
                        
                case RotMatrix: {
                                    port_size.push_back(3);
                                    port_size.push_back(3);
                                }break;
                            
                case PointPos:  {
                                    port_size.push_back(3);
                                    blockfactory::core::Port::Info source_point_port{/*portIndex=*/_in_start_port,
                                                                                    std::vector<int>{3},
                                                                                    blockfactory::core::Port::DataType::DOUBLE};

                                    inputPortInfo.push_back(source_point_port);
                                    _in_start_port = _in_start_port +1;

                                }break;
                case PointAcc: {
                                    port_size.push_back(3);
                                    blockfactory::core::Port::Info source_point_port{/*portIndex=*/_in_start_port,
                                                                                    std::vector<int>{3},
                                                                                    blockfactory::core::Port::DataType::DOUBLE};

                                    inputPortInfo.push_back(source_point_port);       
                                }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized pose components option from parameters selected";
            return false;
        }
        
        blockfactory::core::Port::Info output{/*portIndex=*/i + _out_start_port,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }

        
    return true;
}

bool XBotBlock_Pose::setPoseOut(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
        for(size_t i=0; i < _pose_list.size();i++)
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i + _out_start_port );
            // Check the signal validity
            if (!output) {
                bfError << "Signal not valid";
                return false;
            }
            // verify if the element of the list exists like option
            if(_pose_option.count(_pose_list[i]) > 0)
            {
                switch(_pose_option.at(_pose_list[i]))
                {
                    case Pose:  {
                                    if(_target_frame!="")
                                    {
                                        if(!_model->getPose(_source_frame,_target_frame,_pose_frame))
                                        {
                                            bfError << "Invalid source or target frame";
                                            return false;
                                        }
                                    }
                                    else
                                    {
                                        if(!_model->getPose(_source_frame,_pose_frame))
                                        {
                                            bfError << "Invalid source frame";
                                            return false;
                                        }
                                    }
                                    // Allocate objects for row-major -> col-major conversion
                                    Eigen::Map<Matrix4dSimulink> HomogTransfolMajor(output->getBuffer<double>(),4,4);
                                    
                                    HomogTransfolMajor=_pose_frame.matrix();           
                                }break;
                                
                     case RotMatrix:    {
                                            if(_target_frame!="")
                                            {
                                                if(!_model->getOrientation(_source_frame,_target_frame,_orientation))
                                                {
                                                    bfError << "Invalid source or target frame";
                                                    return false;
                                                }
                                            }
                                            else
                                            {
                                                if(!_model->getOrientation(_source_frame,_orientation))
                                                {
                                                    bfError << "Invalid source frame";
                                                    return false;
                                                }
                                            }
                                            // Allocate objects for row-major -> col-major conversion
                                            Eigen::Map<Matrix3dSimulink> OrientationMajor(output->getBuffer<double>(),3,3);
                                            OrientationMajor=_orientation.matrix();           
                                        }break;
                    case PointPos:  {
                                        // get source point signal
                                        blockfactory::core::InputSignalPtr source_point_sig = blockInfo->getInputPortSignal(/*index=*/ _in_start_port);
                                        
                                            // Check the signal validity
                                        if (!source_point_sig) {
                                            bfError << "Signal not valid";
                                            return false;
                                        }
                                        
                                        // check dimesion of the input signal with the joint number
                                        if(source_point_sig->getWidth() != 3)
                                        {
                                            bfError << "Source point signal must be 3";
                                            return false;
                                        }
                                        
                                        for (size_t  k=0; k < _source_point.size(); ++k)
                                        {
                                            _source_point[k]=source_point_sig->get<double>(k);
                                        }
                                            
                                        if(_target_frame!="")
                                        {
                                            if(!_model->getPointPosition(_source_frame,_target_frame,_source_point,_point_pos))
                                            {
                                                bfError << "Invalid source or target frame";
                                                return false;
                                            }
                                        }
                                        else
                                        {
                                            if(!_model->getPointPosition(_source_frame,_source_point,_point_pos))
                                            {
                                                bfError << "Invalid source frame";
                                                return false;
                                            }
                                        }
                                        // set the signal output
                                        for (size_t  k=0; k < output->getWidth(); ++k)
                                        {
                                            output->set(k, _point_pos[k]);
                                        }
                                    
                                    }break;
                    case PointAcc:  {
                                        size_t add_PointPos_input = 0;
                                        if(_pose_option.count("PointPos") > 0)
                                        {
                                            add_PointPos_input = add_PointPos_input + 1 ;
                                        }
                                        // get source point signal
                                        blockfactory::core::InputSignalPtr point_sig = blockInfo->getInputPortSignal(/*index=*/ 
                                                                                                                    _in_start_port+add_PointPos_input);
                                        
                                            // Check the signal validity
                                        if (!point_sig) {
                                            bfError << "Signal not valid";
                                            return false;
                                        }
                                        
                                        // check dimesion of the input signal with the joint number
                                        if(point_sig->getWidth() != 3)
                                        {
                                            bfError << "Point signal must be 3";
                                            return false;
                                        }
                                        
                                        for (size_t  k=0; k < _point.size(); ++k)
                                        {
                                            _point[k]=point_sig->get<double>(k);
                                        }
                                            
                                        if(!_model->getPointAcceleration(_link_name,_point,_point_acc))
                                        {
                                            bfError << "Invalid source frame";
                                            return false;
                                        }
                                        // set the signal output
                                        for (size_t  k=0; k < output->getWidth(); ++k)
                                        {
                                            output->set(k, _point_acc[k]);
                                        }
                                    
                                    }break;    
                }
            }
            else
            {
                bfError << "Found unrecognized pose components option from parameters selected";
                return false;
            }
        }
    }
    else
    {
        bfError << "Try to perform an operation on a null model";
        return false;
    }

    return true;
}

bool XBotBlock_Pose::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    if(!setPoseOut(blockInfo))
    {
        return false;
    }
    return true;
}


bool XBotBlock_Pose::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(!setPoseOut(blockInfo))
    {
        return false;
    }
    return true;
}
