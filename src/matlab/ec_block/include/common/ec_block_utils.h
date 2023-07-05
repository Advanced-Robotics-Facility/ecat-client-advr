#ifndef EC_BLOCK_UTILS_H
#define EC_BLOCK_UTILS_H

#include <map>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include "ec_utils.h"

class EcBlockUtils
{
    
public:
    
    static XBot::ConfigOptions RetrieveRobotConfig();

    static bool RetrieveModel(const std::string &model_name,
                              bool allow_new_model,
                              uint8_t &model_id,
                              XBot::ModelInterface::Ptr &model,
                              std::string &error_info);
    
    static bool RetrieveRobot(bool allow_new_robot,
                              EcIface::Ptr &robot,
                              std::string &error_info);
    
    static bool RetrieveModelId(uint8_t &model_id);
    
    static bool isValidModelID(uint8_t model_id);
    
    static void clearModelMap();
    static void clearRobot();
    
    static void checkParamSelected(std::string param_selection,std::vector<std::string> &param_selected);
    
private:
    static std::map<std::string,std::map<uint8_t, XBot::ModelInterface::Ptr>> _mdl_map;
    static EcIface::Ptr _client;
};

#endif // EC_BLOCK_UTILS_H
