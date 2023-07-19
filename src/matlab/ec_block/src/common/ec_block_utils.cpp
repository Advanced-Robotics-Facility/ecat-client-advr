#include <common/ec_block_utils.h>
#include <chrono>

EcIface::Ptr EcBlockUtils::_client;
std::vector<int> EcBlockUtils::_joint_id;
std::vector<MR> EcBlockUtils::_motors_ref;
bool EcBlockUtils::_robot_started;

using namespace std::chrono;


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
                                                   
bool EcBlockUtils::retrieve_ec_iface(std::string &error_info,bool start_robot_req)
{
    if(_client == nullptr)
    {
        try{
            auto ec_client_utils=retrieve_cfg();
            _client=ec_client_utils->make_ec_iface();
            
            SSI slave_info;
            if(!_client->retrieve_slaves_info(slave_info))
            {   
#ifndef TEST_MATLAB 
                error_info="Error on get slave information command";
                return false;
#endif
            }
        }catch(std::exception &ex){
            error_info= ex.what();
            return false;
        }
    }
    
    if((start_robot_req)&&(!_robot_started))
    {
        if(!start_robot(error_info))
        {
            return false;
        }
    }
    
    return true;
}

bool EcBlockUtils::start_robot(std::string &error_info)
{
    int ctrl_mode;
    std::vector<float> gains;
    std::vector<double> q_home,q_trj;
    
    _joint_id.clear();
    retrieve_motor_info(_joint_id,ctrl_mode,gains,q_home,q_trj,error_info);
    
    MotorStatusMap motors_status_map;
    FtStatusMap ft6_status_map;
    ImuStatusMap imu_status_map;
    PwrStatusMap pow_status_map;

    if(ec_sense(motors_status_map,ft6_status_map,imu_status_map,pow_status_map,_motors_ref))
    {
        if(motors_status_map.empty())
        {
            error_info = "EtherCAT client sense failed";
            return false;
        }
    }
    else
    {
        error_info = "EtherCAT Client not alive!";
        return false;
    }
    
    _motors_ref.clear();
    for(size_t i=0; i < _joint_id.size();i++)
    {
        auto id = _joint_id[i];
        if(motors_status_map.count(id) >0)
        {
            auto motor_pos = std::get<1>(motors_status_map[id]);
            _motors_ref.push_back(std::make_tuple(id,ctrl_mode,motor_pos,0.0,0.0,gains[0],gains[1],gains[2],gains[3],gains[4],1,0,0));
            
            //motors_start.push_back(std::make_tuple(id,ctrl_mode,gains));
            //brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
        }
    }   
    
    
    MST motors_start = {};
    PAC brake_cmds = {};

    _robot_started = true;
    if(!motors_start.empty())
    {
        if(_client->start_motors(motors_start))
        {
            if(!brake_cmds.empty())
            {
                if(!_client->pdo_aux_cmd(brake_cmds))
                {
                    error_info= "Cannot perform the release brake command of the motors";
                    _robot_started=false;
                }
                else
                {
                    std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                    if(!_client->pdo_aux_cmd_sts(brake_cmds))
                    {
                        error_info= "Error on brake status on the motors";
                        _robot_started=false;
                    }
                }
            }
        }
        else
        {
            error_info= "Motors not started";
            _robot_started=false;
        }
    }

    return _robot_started;
}


void EcBlockUtils::stop_robot()
{
    if(_robot_started)
    {
        _robot_started=false;
        PAC brake_cmds = {};

        for(size_t i=0; i < _joint_id.size();i++)
        {
            auto id = _joint_id[i];
            brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
        }
        
    //     if(!brake_cmds.empty())
    //     {
    //         if(_client->pdo_aux_cmd(brake_cmds))
    //         {
    //             std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
    //             if(_client->pdo_aux_cmd_sts(brake_cmds))
    //             {
    //                 _client->stop_motors();
    //             }
    //         }
    //     }
    }

}

void EcBlockUtils::stop_ec_iface()
{
    if(_client != nullptr)
    {
        stop_robot();
        if(_client->is_client_alive())
        {
            _client->stop_client();
        }
    }
    _client.reset();
}

void EcBlockUtils::retrieve_motor_info(std::vector<int> &joint_id,
                                       int &ctrl_mode,
                                       std::vector<float> &gains,
                                       std::vector<double> &q_home,
                                       std::vector<double> &q_trj,
                                       std::string &error_info)
{
    try{
        auto ec_client_utils=retrieve_cfg();
        
        auto home_map= ec_client_utils->get_ec_cfg().homing_position;
        auto trj_map = ec_client_utils->get_ec_cfg().trajectory;
        
        ctrl_mode= ec_client_utils->get_ec_cfg().control_mode_type;
        gains= ec_client_utils->get_ec_cfg().gains;

        for ( auto &[id, home_pos] : home_map) {
            joint_id.push_back(id);
            q_home.push_back(home_pos);
            q_trj.push_back(trj_map[id]);
        }
    }catch(std::exception &ex){
        error_info= ex.what();
    }

}

void EcBlockUtils::retrieve_imu_info(std::vector<int> &imu_id,std::string &error_info)
{
    try{
        auto ec_client_utils=retrieve_cfg();
        imu_id = ec_client_utils->get_ec_cfg().imu_id;
    }catch(std::exception &ex){
        error_info= ex.what();
    }
}

void EcBlockUtils::retrieve_ft_info(std::vector<int> &ft_id,std::string &error_info)
{
    try{
        auto ec_client_utils=retrieve_cfg();
        ft_id = ec_client_utils->get_ec_cfg().ft_id;
    }catch(std::exception &ex){
        error_info= ex.what();
    }
}

void EcBlockUtils::retrieve_pow_info(std::vector<int> &pow_id,std::string &error_info)
{
    try{
        auto ec_client_utils=retrieve_cfg();
        pow_id = ec_client_utils->get_ec_cfg().pow_id;
    }catch(std::exception &ex){
        error_info= ex.what();
    }
}

bool EcBlockUtils::ec_sense(MotorStatusMap &motors_status_map,
                            FtStatusMap &ft6_status_map,
                            ImuStatusMap &imu_status_map,
                            PwrStatusMap &pow_status_map,
                            std::vector<MR> &motors_ref)
{
    
    if(!_client->is_client_alive())
    {
        return false;
    }
    
    motors_status_map.clear();
    motors_status_map=_client->get_motors_status();
    
    if(motors_status_map.empty())
    {
#ifdef TEST_MATLAB 
        for(int i=0; i < _joint_id.size();i++)
        {
            int q_id = _joint_id[i];
            motors_status_map[q_id] = std::make_tuple(0,0,0,0,0,25,26,3,0,0,0,2);
        }
#endif
    }
    
    ft6_status_map.clear();
    ft6_status_map = _client->get_ft6_status();

    if(ft6_status_map.empty())
    {
#ifdef TEST_MATLAB 
        std::vector<int> ft_id_v;
        std::string error_info="";
        retrieve_ft_info(ft_id_v,error_info);
        for(int i=0; i < ft_id_v.size();i++)
        {
            int ft_id = ft_id_v[i];
            float value_rand = (float) ft_id;
            ft6_status_map[ft_id] = {value_rand,120,130,20,25,30};
        }
#endif
    }
    
    imu_status_map.clear();
    imu_status_map = _client->get_imu_status();
    
    if(imu_status_map.empty())
    {
#ifdef TEST_MATLAB 
        std::vector<int> imu_id_v;
        std::string error_info="";
        retrieve_imu_info(imu_id_v,error_info);
        for(int i=0; i < imu_id_v.size();i++)
        {
            int imu_id = imu_id_v[i];
            float value_rand = (float) imu_id;
            imu_status_map[imu_id] = {value_rand,15.0,20.0,5.0,2.0,3.0,0,0,0,1};
        }
#endif
    }
    
    pow_status_map.clear();
    pow_status_map = _client->get_pow_status();
    
    if(pow_status_map.empty())
    {
#ifdef TEST_MATLAB 
        std::vector<int> pow_id_v;
        std::string error_info="";
        retrieve_pow_info(pow_id_v,error_info);
        for(int i=0; i < pow_id_v.size();i++)
        {
            int pow_id = pow_id_v[i];
            pow_status_map[pow_id] = {48.0,47.5,5.0,20,25,30};
        }
#endif
    }
    
    motors_ref = _motors_ref;
    
    return true;
}

bool EcBlockUtils::ec_move(MotorRefFlags flag,std::vector<MR> motors_ref)
{
    if(!_client->is_client_alive())
    {
        return false;
    }
    
    _motors_ref = motors_ref;
    _client->set_motors_references(flag,_motors_ref);
    
    return true;
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
