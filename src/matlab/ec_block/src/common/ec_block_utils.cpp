#include <common/ec_block_utils.h>


std::map<std::string,std::map<uint8_t, XBot::ModelInterface::Ptr>> EcBlockUtils::_mdl_map;
EcIface::Ptr EcBlockUtils::_client;

XBot::ConfigOptions EcBlockUtils::RetrieveRobotConfig()
{
 
    char* ec_client_cfg_path;
    ec_client_cfg_path = getenv ("EC_CLIENT_CFG");
    XBot::ConfigOptions config; 

    if (ec_client_cfg_path==NULL)
    {
        throw std::runtime_error("EC Client configuration not found, setup the environment variable: EC_CLIENT_CFG ");
    }
    else
    {
        config=XBot::ConfigOptions::FromConfigFile(ec_client_cfg_path); 
    }
        
    return config;

}
                                                   
bool EcBlockUtils::RetrieveModel(const std::string &model_name,
                                 bool allow_new_model,
                                 uint8_t &model_id,
                                 XBot::ModelInterface::Ptr &model,
                                 std::string &error_info)
{
    std::map<uint8_t, XBot::ModelInterface::Ptr> mdl_id_map;

    if (!(_mdl_map.count(model_name)>0))
    {
        if(allow_new_model)
        {
            try{
                
                if(RetrieveModelId(model_id))
                {
                    XBot::ConfigOptions config = RetrieveRobotConfig();
            
                    model=XBot::ModelInterface::getModel(config);
                    
                    mdl_id_map[model_id]=model;
                    
                    _mdl_map[model_name]=mdl_id_map;
                }
                else
                {
                    error_info = "reached maximum number of models allowed";
                    return false;
                }
                    
                
            }catch(std::exception &ex){
                error_info = ex.what();
                return false;
            }
        }
        else
        {
            error_info = "model not found"; 
            return false;
        }
        
    }

    mdl_id_map.clear();
    mdl_id_map = _mdl_map.at(model_name) ;

    if(!mdl_id_map.empty())
    {
        model_id = mdl_id_map.begin()->first; 
        
        model.reset();
        model = mdl_id_map.begin()->second;
    }
    else
    {
        error_info = "got an empty model id map"; 
        return false;
    }

    return true;
}

bool EcBlockUtils::RetrieveModelId(uint8_t &model_id)
{
    auto new_model_id =  _mdl_map.size() + 1;

    uint8_t max_model_number = std::numeric_limits<uint8_t>::max();
    
    if(new_model_id > max_model_number)
    {
        model_id= max_model_number; // Max Models number = 255
        return false;
    }
    else
    {
        model_id= (uint8_t) new_model_id;
        return true;
    }
}

void EcBlockUtils::clearModelMap()
{
    _mdl_map.clear();
}

bool EcBlockUtils::RetrieveRobot(bool allow_new_robot,
                                 EcIface::Ptr &robot,
                                 std::string &error_info)
{
    if(_client == nullptr)
    {
        if(allow_new_robot)
        {
            char* ec_client_cfg_path;
            ec_client_cfg_path = getenv ("EC_CLIENT_CFG");

            if (ec_client_cfg_path==NULL)
            {
                error_info= "EC Client configuration not found, setup the environment variable: EC_CLIENT_CFG";
                return false;
            }
            else
            {
                try{
                    auto ec_client_cfg_file = YAML::LoadFile(ec_client_cfg_path);
                    EcUtils::Ptr  ec_client_utils=std::make_shared<EcUtils>(ec_client_cfg_file);
                    EcIface::Ptr client=ec_client_utils->make_ec_iface();
                }catch(std::exception &ex){
                    error_info= ex.what();
                    return false;
                }
            }
        }
        else
        {
            error_info = "robot not found";
            return false;
        }
            
    }
    
    robot = _client;
    return true;
}


void EcBlockUtils::clearRobot()
{
    if(_client != nullptr)
    {
        if(_client->is_client_alive())
        {
            _client->stop_client();
        }
    }
}


void EcBlockUtils::checkParamSelected(std::string param_selection,std::vector<std::string> &param_selected)
{
    param_selected.clear();
    
    if(!param_selection.empty())
    {
        std::string chars = " '{}";
        for (char c: chars) 
        {
            param_selection.erase(std::remove(param_selection.begin(), param_selection.end(), c), param_selection.end());
        }
        if(!param_selection.empty())
        {
            size_t pos = 0;
            std::string delimiter = ",";
            while ((pos = param_selection.find(delimiter)) != std::string::npos) {
                std::string item= param_selection.substr(0, pos);
                param_selected.push_back(item);
                param_selection.erase(0, pos + delimiter.length());
            }
            param_selected.push_back(param_selection);
        }
    }
}
