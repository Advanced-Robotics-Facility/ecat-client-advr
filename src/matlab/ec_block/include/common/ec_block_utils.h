#ifndef EC_BLOCK_UTILS_H
#define EC_BLOCK_UTILS_H

#include <map>
#include "ec_utils.h"

class EcBlockUtils
{
    
public:
    
    static char* retrieve_cfg_path();
    static EcUtils::Ptr retrieve_cfg();
    static bool retrieve_ec_iface(std::string &error_info);
    static void stop_ec_iface();
    static void retrieve_ec_info(int &joint_numb,
                                std::vector<int> &joint_id,
                                int &ctrl_mode,
                                std::vector<float> &gains,
                                std::vector<double> &q_home,
                                std::vector<double> &q_trj,
                                std::string &error_info);
    
    static bool ec_sense(MotorStatusMap &motors_status_map,
                         FtStatusMap &ft6_status_map,
                         ImuStatusMap &imu_status_map,
                         PwrStatusMap &pow_status_map,
                         std::vector<MR> &motors_ref);
    
    static bool ec_move(MotorRefFlags flag,std::vector<MR> motors_ref);
    
    
    static void check_param_selected(std::string param_selection,std::vector<std::string> &param_selected);
    
private:
    static EcIface::Ptr _client;
    static std::vector<int> _joint_id;
    static std::vector<MR> _motors_ref;
    
    static bool init_robot(std::string &error_info);
    static void stop_robot();
};

#endif // EC_BLOCK_UTILS_H
