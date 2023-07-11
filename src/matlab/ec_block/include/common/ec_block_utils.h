#ifndef EC_BLOCK_UTILS_H
#define EC_BLOCK_UTILS_H

#include <map>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include "ec_utils.h"

class EcBlockUtils
{
    
public:
    
    static char* retrieve_cfg_path();
    static EcUtils::Ptr retrieve_cfg();

    static bool RetrieveModel(const std::string &model_name,
                              bool allow_new_model,
                              uint8_t &model_id,
                              XBot::ModelInterface::Ptr &model,
                              std::string &error_info);
    
    static bool retrieve_robot(EcIface::Ptr &robot,std::string &error_info);
    
    static std::vector<float> retrieve_joint_gains();
    static std::vector<int> retrive_joint_id();
    static int retrive_ctrl_mode();
    static int retrive_joint_numb(std::string &error_info);
    
    static bool RetrieveModelId(uint8_t &model_id);
    
    static bool isValidModelID(uint8_t model_id);
    
    static void clearModelMap();
    static void clear_robot();
    
    static void check_param_selected(std::string param_selection,std::vector<std::string> &param_selected);
    
private:
    static std::map<std::string,std::map<uint8_t, XBot::ModelInterface::Ptr>> _mdl_map;
    static EcIface::Ptr _client;
    static std::vector<int> _joint_id;
};

#endif // EC_BLOCK_UTILS_H
