#include "XBotBlock/Utilities/COM.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

using namespace XBB_COM;

XBotBlock_COM::XBotBlock_COM(XBot::ModelInterface::Ptr model,
                             std::string reference_frame,
                             std::vector<std::string> COM_list,
                             size_t start_port):
_model(model),
_reference_frame(reference_frame),
_COM_list(COM_list),
_start_port(start_port)
{
    // create class and save the ModelInterface ptr, list and start port (input and output)
    // initialize joint_number to one
    _n_joint = 1;
    
    //override joint_number
    if(_model!= nullptr)
    {
        _n_joint = _model->getJointNum();
    }
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_COM::configureSizeAndPorts(blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    
    for(size_t i=0; i < _COM_list.size();i++)
    {
        std::vector<int> port_size;
        if(_COM_option.count(_COM_list[i]) > 0)
        {
            switch(_COM_option.at(_COM_list[i]))
            {
                case p_COM: {
                                port_size.push_back(3);
                            }break;
                case v_COM: {
                                port_size.push_back(3);
                            }break;
                case a_COM: {
                                port_size.push_back(3);
                            }break;
                case CM:    {
                                port_size.push_back(6);
                            }break;
                case CM_Matrix: {
                                    port_size.push_back(6);
                                    port_size.push_back(_n_joint);
                                }break;
                case CMMdotQdot:    {
                                        port_size.push_back(6);
                                    }break;
            }
            
        }
        else
        {
            bfError << "Found unrecognized COM components option from parameters selected";
            return false;
        }
        
        blockfactory::core::Port::Info output{/*portIndex=*/i + _start_port,
                                              port_size,
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(output);
    }
        
    return true;
}

bool XBotBlock_COM::setCOMOut(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
        for(size_t i=0; i < _COM_list.size();i++)
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output= blockInfo->getOutputPortSignal(/*index=*/i + _start_port );
            // Check the signal validity
            if (!output) {
                bfError << "Signal not valid";
                return false;
            }
            // verify if the element of the list exists like option
            if(_COM_option.count(_COM_list[i]) > 0)
            {
                // save into auxiliary vector the COM information
                Eigen::VectorXd aux_vector;
                switch(_COM_option.at(_COM_list[i]))
                {
                    case p_COM: {
                                    Eigen::Vector3d aux_3dvector;
                                    if(_reference_frame!="")
                                    {
                                        if(!_model->getCOM(_reference_frame,aux_3dvector))
                                        {
                                            bfError << "Invalid reference frame ";
                                            return false;
                                        }
                                    }
                                    else
                                    {
                                        _model->getCOM(aux_3dvector);
                                    }
                                    aux_vector=aux_3dvector;
                                    
                                }break;
                    
                    case v_COM: {
                                    Eigen::Vector3d aux_3dvector;
                                    _model->getCOMVelocity(aux_3dvector);  
                                    aux_vector=aux_3dvector;
                                }break;
                    case a_COM: {
                                    Eigen::Vector3d aux_3dvector;
                                    _model->getCOMAcceleration(aux_3dvector);
                                    aux_vector=aux_3dvector;
                                }break;
                    case CM:    {
                                     Eigen::Vector6d aux_6dvector;
                                    _model->getCentroidalMomentum(aux_6dvector);
                                    aux_vector=aux_6dvector;
                                }break;
                    case CM_Matrix: {
                                        Eigen::MatrixXd cm_matrix;
                                        _model->getCentroidalMomentumMatrix(cm_matrix);
                                        // Allocate objects for row-major -> col-major conversion
                                        Eigen::Map<MatrixXdSimulink> CMMatrixColMajor(output->getBuffer<double>(),6,_n_joint);
                                        CMMatrixColMajor=cm_matrix;
                                    }break;
                    case CMMdotQdot:    {
                                            Eigen::MatrixXd cm_matrix;
                                            Eigen::Vector6d aux_6dvector;
                                            _model->getCentroidalMomentumMatrix(cm_matrix,aux_6dvector);
                                            aux_vector=aux_6dvector;
                                        }break;
                }
            
                if(_COM_list[i]!="CM_Matrix")
                {
                    // check the auxiliary vector size with output signal size
                    if(aux_vector.size() != output->getWidth())
                    {
                        bfError << "Different dimension of output port: " + std::to_string(output->getWidth()) +" and ouput vector: " + std::to_string(aux_vector.size());
                        return false;
                    }
                    
                    // set the signal output
                    for (size_t  k=0; k < output->getWidth(); ++k)
                    {
                        output->set(k, aux_vector[k]);
                    }
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

bool XBotBlock_COM::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    if(!setCOMOut(blockInfo))
    {
        return false;
    }
    return true;
}


bool XBotBlock_COM::output(const blockfactory::core::BlockInformation* blockInfo)
{
    
    if(!setCOMOut(blockInfo))
    {
        return false;
    }
    return true;
    
}
