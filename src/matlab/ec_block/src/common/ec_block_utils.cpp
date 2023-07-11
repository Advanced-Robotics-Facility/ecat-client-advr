#include <common/ec_block_utils.h>


std::map<std::string,std::map<uint8_t, XBot::ModelInterface::Ptr>> EcBlockUtils::_mdl_map;
EcIface::Ptr EcBlockUtils::_client;
std::vector<int> EcBlockUtils::_joint_id;

char* EcBlockUtils::retrieve_cfg_path()
{
    char* ec_client_cfg_path;
    ec_client_cfg_path = getenv ("EC_CLIENT_CFG");
    
    if (ec_client_cfg_path==NULL)
    {
        throw std::runtime_error("EC Client configuration not found, setup the environment variable: EC_CLIENT_CFG ");
    }
    
    return ec_client_cfg_path;
}

EcUtils::Ptr EcBlockUtils::retrieve_cfg()
{
    EcUtils::Ptr ec_client_utils;
     try{
        auto ec_client_cfg_path = retrieve_cfg_path(); 
        auto ec_client_cfg_file = YAML::LoadFile(ec_client_cfg_path);
        ec_client_utils=std::make_shared<EcUtils>(ec_client_cfg_file);
    }catch(std::exception &ex){
        throw std::runtime_error(ex.what());
    }
    return ec_client_utils;
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
                    auto ec_client_cfg_path = retrieve_cfg_path();
                    
                    XBot::ConfigOptions config = XBot::ConfigOptions::FromConfigFile(ec_client_cfg_path); 
            
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

bool EcBlockUtils::retrieve_robot(EcIface::Ptr &robot,std::string &error_info)
{
    if(_client == nullptr)
    {
        try{
            auto ec_client_utils=retrieve_cfg();
            _client=ec_client_utils->make_ec_iface();
            
            SSI slave_info;
            if(_client->retrieve_slaves_info(slave_info))
            {   
                if(!slave_info.empty())
                {
                    for ( auto &[id, type, pos] : slave_info ) {
                        if((type==0x15) || (type==0x12))//HP or LP motor
                        {
                            //_joint_id.push_back(id);
                        }
                    }
                }
            }
        }catch(std::exception &ex){
            error_info= ex.what();
            return false;
        }
    }
    
    robot = _client;
    return true;
}

int EcBlockUtils::retrive_ctrl_mode()
{
    int ctrl_mode;
    try{
        auto ec_client_utils=retrieve_cfg();
        ctrl_mode= ec_client_utils->get_ec_cfg().control_mode_type;
    }catch(std::exception &ex){
    }
    return ctrl_mode;
}

std::vector<int> EcBlockUtils::retrive_joint_id()
{
    return _joint_id;
}

std::vector<float> EcBlockUtils::retrieve_joint_gains()
{
    std::vector<float> gains;
    try{
        auto ec_client_utils=retrieve_cfg();
        gains= ec_client_utils->get_ec_cfg().gains;
    }catch(std::exception &ex){
    }
    return gains;
}

int EcBlockUtils::retrive_joint_numb(std::string &error_info)
{
    int joint_numb=1;
    try{
        auto ec_client_utils=retrieve_cfg();
        joint_numb= ec_client_utils->get_ec_cfg().homing_position.size();
        for ( auto &[id, home_pos] : ec_client_utils->get_ec_cfg().homing_position ) {
            _joint_id.push_back(id);
        }
    }catch(std::exception &ex){
        error_info= ex.what();
    }
    return joint_numb;
}

void EcBlockUtils::clear_robot()
{
    if(_client != nullptr)
    {
        if(_client->is_client_alive())
        {
            _client->stop_client();
        }
    }
    _client.reset();
}


void EcBlockUtils::check_param_selected(std::string param_selection,std::vector<std::string> &param_selected)
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
