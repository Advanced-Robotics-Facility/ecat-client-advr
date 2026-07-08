#ifndef __EC_UTILS__
#define __EC_UTILS__

#include <yaml-cpp/yaml.h>
#include <thread>
#include "protocol/udp/ec_udp.h"
#include "protocol/tcp/ec_tcp.h"
#include "protocol/ipc/iddp/ec_iddp.h"
#include "protocol/ipc/zipc/ec_zipc.h"
#include <ecat_core/trajectory.h>

enum class TrjType : uint8_t
{
    zero,
    start,
    trj1,
    trj2
};

enum class LimitPolicy
{
    NONE,
    MARGIN,
    SCALE
};

typedef struct TRJ_INFO_t{
    std::string type;
    std::vector<std::string> limits;
    std::vector<double> adjustment;
    LimitPolicy adjustment_type = LimitPolicy::NONE;
}TRJ_INFO;


using trj_info_map = std::map<int, TRJ_INFO>;
extern const std::map<std::string, trj_info_map> trj_type_map;

typedef struct ESC_TRJ_t{

    int32_t esc_id;
    
    double trj1;
    double trj2;
    double set_zero;
    std::vector<double> trj_limit;

    double start;
    double set_ref;
    double set_trj;

    Trj_ptr general_trj;

    bool check_limit(double v);
    void set_trj_limit(const std::vector<double>& limits);
    void setup_trj(TrjType type);
    void set_target(double ref);
}ESC_TRJ;

class EcUtils
{
public:
    
    typedef struct DEVICE_CONFIG_t{
        std::string device_name;
        uint32_t type;
        int control_mode_type;
        std::vector<float> gains;
        bool brake_present; // only for motor
    }DEVICE_CONFIG;

    typedef struct TRAJECTORY_CONFIG_t{
        std::map<int,std::map<std::string,double>> set_point;
        std::map<int,double> homing;
        std::map<int,double> trajectory;
        std::map<int,std::map<std::string,Trj_ptr>> trj_generator;
    }TRAJECTORY_CONFIG_t;
        
    typedef struct EC_CONFIG_t{
        
        std::string protocol;
        std::string host_name;
        uint32_t host_port;
        int period_ms; 
        bool logging;

        std::map<int,DEVICE_CONFIG> device_config_map;
        
        std::string trj_type="";
        float trj_time=0;
        int repeat_trj=1;
        std::map<std::string,TRAJECTORY_CONFIG_t> trj_config_map;
        SSI fake_slave_info;
        
    }EC_CONFIG;
    
    typedef std::shared_ptr<EcUtils> Ptr;
    
    EcUtils(EC_CONFIG ec_cfg);
    EcUtils();
    
    ~EcUtils();
    
    EC_CONFIG get_ec_cfg();
    std::string get_ec_cfg_file();
    EcIface::Ptr make_ec_iface(); // EtherCAT Client Interface
private:
    
    std::string _control_mode;
    EC_CONFIG _ec_cfg;
    std::string _ec_cfg_file;
    
    void compute_absolute_path(std::string dir_path,std::string &file_path);
    void generate_fake_slave_info();
    void device_config_map(const YAML::Node & device_config_node,std::string device_type);
    void config_trajectory();      
    void config_device();
    void trajectory_generator();
    
    std::vector<std::string> _device_type_vector={"motor","valve","pump"};  
    std::vector<std::string> _robot_path_v={"robot_id_map_path","robot_control_path"};
    std::vector<std::string> _robot_abs_path={"",""};
    YAML::Node _robot_id_map_node;
    YAML::Node _robot_control_node;
};

#endif
