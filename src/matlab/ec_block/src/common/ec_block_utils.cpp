#include <common/ec_block_utils.h>
#include <chrono>

EcIface::Ptr EcBlockUtils::_client;
std::vector<int> EcBlockUtils::_joint_id;
std::vector<MR> EcBlockUtils::_motors_ref;

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
                                                   
bool EcBlockUtils::retrieve_ec_iface(std::string &error_info)
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
                    for(int i=0; i < _joint_id.size();i++)
                    {
                        bool found_motor=false;
                        int q_id = _joint_id[i];
                        for ( auto &[id, type, pos] : slave_info ) {
                            if(q_id == id)
                            {
                                found_motor = true;
                                break;
                            }
                        }
                        if(!found_motor)
                        {
                            error_info = "Motort id= "+ std::to_string(q_id) + " not found in the robot";
                            return false;
                        }
                    }
                }
            }

            if(!init_robot(error_info))
            {
                return false;
            }
            
        }catch(std::exception &ex){
            error_info= ex.what();
            return false;
        }
    }
    
    return true;
}

bool EcBlockUtils::init_robot(std::string &error_info)
{
    int joint_number,ctrl_mode;
    std::vector<float> gains;
    std::vector<double> q_home,q_trj;
    
    _joint_id.clear();
    retrieve_ec_info(joint_number,_joint_id,ctrl_mode,gains,q_home,q_trj,error_info);

    MST motors_start = {};
    PAC brake_cmds = {};
    
    MotorStatusMap motors_status_map;
    FtStatusMap ft6_status_map;
    ImuStatusMap imu_status_map;
    PwrStatusMap pow_status_map;

    if(ec_sense(motors_status_map,ft6_status_map,imu_status_map,pow_status_map,_motors_ref))
    {
        if(motors_status_map.empty())
        {
            error_info = "Robot not initialized, got an empty motors map status";
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

    bool robot_init = true;
    if(!motors_start.empty())
    {
        if(_client->start_motors(motors_start))
        {
            if(!brake_cmds.empty())
            {
                if(!_client->pdo_aux_cmd(brake_cmds))
                {
                    error_info= "Cannot perform the release brake command of the motors";
                    robot_init=false;
                }
                else
                {
                    std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                    if(!_client->pdo_aux_cmd_sts(brake_cmds))
                    {
                        error_info= "Error on brake status on the motors";
                        robot_init=false;
                    }
                }
            }
        }
        else
        {
            error_info= "Motors not started";
            robot_init=false;
        }
    }

    return robot_init;
}

void EcBlockUtils::stop_robot()
{
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

void EcBlockUtils::retrieve_ec_info(int &joint_numb,
                                    std::vector<int> &joint_id,
                                    int &ctrl_mode,
                                    std::vector<float> &gains,
                                    std::vector<double> &q_home,
                                    std::vector<double> &q_trj,
                                    std::string &error_info)
{
    joint_numb=1;
    
    try{
        auto ec_client_utils=retrieve_cfg();
        
        auto home_map= ec_client_utils->get_ec_cfg().homing_position;
        auto trj_map = ec_client_utils->get_ec_cfg().trajectory;
        
        joint_numb= home_map.size();
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
    
    ft6_status_map.clear();
    ft6_status_map = _client->get_ft6_status();
    
    imu_status_map.clear();
    imu_status_map = _client->get_imu_status();
    
    pow_status_map.clear();
    pow_status_map = _client->get_pow_status();

    if(motors_status_map.empty())
    {
#ifdef TEST_MATLAB 
        for(int i=0; i < _joint_id.size();i++)
        {
            int q_id = _joint_id[i];
            motors_status_map[q_id] = std::make_tuple(0,0.0,0,0,0,0,0,0,0,0,0,0);
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
