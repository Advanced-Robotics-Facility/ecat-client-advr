#include <common/ec_block_utils.h>
#include <chrono>

EcIface::Ptr EcBlockUtils::_client;
std::vector<int> EcBlockUtils::_joint_id;
MotorReferenceMap EcBlockUtils::_motors_ref;
bool EcBlockUtils::_robot_started;

using namespace std::chrono;


EcUtils::Ptr EcBlockUtils::retrieve_cfg()
{
    EcUtils::Ptr ec_client_utils;
     try{
        ec_client_utils=std::make_shared<EcUtils>();
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
    for(const auto &id:_joint_id)
    {
        if(motors_status_map.count(id) >0)
        {
            auto motor_pos = std::get<1>(motors_status_map[id]);
            _motors_ref[id]=std::make_tuple(ctrl_mode,motor_pos,0.0,0.0,gains[0],gains[1],gains[2],gains[3],gains[4],1,0,0);
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

        for (const auto &id : _joint_id)
        {
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
        _client->stop_client();
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
       
        auto motor_id_v = ec_client_utils->get_ec_cfg().motor_id;
        auto home_map= ec_client_utils->get_ec_cfg().homing_position;
        auto trj_map = ec_client_utils->get_ec_cfg().trajectory;
        
//         ctrl_mode= ec_client_utils->get_ec_cfg().control_mode_type;
//         gains= ec_client_utils->get_ec_cfg().gains;

        for (const auto &motor_id : motor_id_v){
            joint_id.push_back(motor_id);
            q_home.push_back(home_map[motor_id]);
            q_trj.push_back(trj_map[motor_id]);
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
                            MotorReferenceMap &motors_ref)
{
    
    if(!_client->get_client_status().run_loop)
    {
        return false;
    }
    
    motors_status_map.clear();
    _client->get_motors_status(motors_status_map);
    
    ft6_status_map.clear();
    _client->get_ft_status(ft6_status_map);
    
    imu_status_map.clear();
    _client->get_imu_status(imu_status_map);

    
    pow_status_map.clear();
    _client->get_pow_status(pow_status_map);
    
    
    motors_ref = _motors_ref;
    
    return true;
}

bool EcBlockUtils::ec_move(RefFlags flag,MotorReferenceMap motors_ref)
{
    if(!_client->get_client_status().run_loop)
    {
        return false;
    }
    
    _motors_ref = motors_ref;
    _client->set_motors_references(_motors_ref);
    
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
