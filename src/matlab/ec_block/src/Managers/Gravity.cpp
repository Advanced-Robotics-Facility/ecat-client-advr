#include "XBotBlock/Managers/Gravity.h"
#include <XBotBlock/Common/Utils.h>

#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Signal.h>

using namespace XBB_Gravity;

using Matrix4dSimulink = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
using Matrix3dSimulink = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;


XBotBlock_Gravity::XBotBlock_Gravity(XBot::ModelInterface::Ptr model,
                                     std::string gravity_request,
                                     std::string reference_frame,
                                     size_t in_start_port,
                                     size_t out_start_port) :
_model(model),
_gravity_request(gravity_request),
_reference_frame(reference_frame),
_in_start_port(in_start_port),
_out_start_port(out_start_port)
{
    // create class and save the XBotInterface ptr, list and start port (input and output)
}

// Keep in mind that after this step, all the allocated memory will be deleted.
// Memory persistency is guaranteed starting from the initialize() method.
bool XBotBlock_Gravity::configureSizeAndPorts(blockfactory::core::InputPortsInfo &inputPortInfo,
                                              blockfactory::core::OutputPortsInfo &outputPortInfo)
{
    bool gravity_request_recognized=false;
    if((_gravity_request =="Get")||(_gravity_request =="Both"))
    {
        gravity_request_recognized=true;
        blockfactory::core::Port::Info ouput{/*portIndex=*/_out_start_port,
                                              std::vector<int>{3},
                                              blockfactory::core::Port::DataType::DOUBLE};
        outputPortInfo.push_back(ouput);

    }
    
    if((_gravity_request =="Set")||(_gravity_request =="Both"))
    {
        gravity_request_recognized=true;
        blockfactory::core::Port::Info input{/*portIndex=*/_in_start_port,
                                            std::vector<int>{3},
                                            blockfactory::core::Port::DataType::DOUBLE};
        inputPortInfo.push_back(input);
    }
    
    if((!gravity_request_recognized)&&(_gravity_request !="Off"))
    {
        bfError << "Found unrecognized gravity request";
        return false;
    }
    
    return true;
}

bool XBotBlock_Gravity::gravity(const blockfactory::core::BlockInformation* blockInfo)
{
    //check model is a null pointer
    if (_model != nullptr)
    {
        if((_gravity_request =="Set")||(_gravity_request =="Both"))
        {
            // Get general signal input
            blockfactory::core::InputSignalPtr gravity_signal = blockInfo->getInputPortSignal(/*index=*/_in_start_port);
            
            // Check the signal validity
            if (!gravity_signal) {
                bfError << "gravity input signal not valid";
                return false;
            }
            
            for (size_t  k=0; k < _set_gravity.size(); ++k)
            {
                _set_gravity[k]=gravity_signal->get<double>(k);
            }
        
            if(_reference_frame!="")
            {
                if(!_model->setGravity(_reference_frame,_set_gravity))
                {
                    bfError << "Invalid reference frame";
                    return false;
                }
            }
            else
            {
                _model->setGravity(_set_gravity);
            }
            
            // model update
            _model->update();
        }
        
        // set all ouput of the list
        if((_gravity_request =="Get")||(_gravity_request =="Both"))
        {
            // get ouput signal
            blockfactory::core::OutputSignalPtr output_signal= blockInfo->getOutputPortSignal(/*index=*/_out_start_port);
            // Check the signal validity
            if (!output_signal) {
                bfError << "Signal not valid";
                return false;
            }
            
            if(_reference_frame!="")
            {
                if(!_model->getGravity(_reference_frame,_get_gravity))
                {
                    bfError << "Invalid reference frame ";
                    return false;
                }
            }
            else
            {
                _model->getGravity(_get_gravity);
            }
            
            for (size_t  k=0; k < _get_gravity.size(); ++k)
            {
                output_signal->set(k, _get_gravity[k]);
            }
        }
    }

    return true;
}


bool XBotBlock_Gravity::initialize(blockfactory::core::BlockInformation* blockInfo)
{
    if(!gravity(blockInfo))
    {
        return false;
    }
    
    return true;
}


bool XBotBlock_Gravity::output(const blockfactory::core::BlockInformation* blockInfo)
{
    if(!gravity(blockInfo))
    {
        return false;
    }
    
    return true;
}
