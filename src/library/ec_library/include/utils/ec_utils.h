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

    bool check_limit(double v){

        if (trj_limit.size() == 1){
            if (std::abs(v) > trj_limit[0]){
                DPRINTF("Trajectory value %.3f on device id %d exceeds limit ±%.3f\n", v, esc_id, trj_limit[0]);
                return false;
            }
        }
        else if (trj_limit.size() == 2){
            if (v < trj_limit[0] || v > trj_limit[1]){
                DPRINTF("Trajectory value %.3f on device id %d is outside allowed range [%.3f, %.3f]\n",
                         v, esc_id, trj_limit[0], trj_limit[1]);
                return false;
            }
        }
        else{
            DPRINTF("Invalid trj_limit size\n");
            return false;
        }
        return true;
    }

    void check_trj_limit(){
       
        if (trj_limit.empty()) return;

        for (double v : {trj1, trj2}){
            if(!check_limit(v)){
                throw std::runtime_error("Trajectory limit error!");
            }
        }
    }

    void set_target(double ref){
        
        if (!trj_limit.empty()){
            if(!check_limit(ref)){
                return;
            }
        }

        set_ref = ref;
    }

    void setup_trj(TrjType type){
        switch (type){
        case TrjType::zero: set_trj = set_zero; break;
        case TrjType::start: set_trj = start; break;
        case TrjType::trj1: set_trj = trj1; break;
        case TrjType::trj2: set_trj = trj2; break;

        default:
            throw std::runtime_error("Error in setup_trj function, trajectory type not recognized!");
        }

        start = set_ref;
    }

}ESC_TRJ;


enum class LimitPolicy
{
    NONE,
    MARGIN,
    SCALE
};

typedef struct TRJ_INFO_t{
    std::string type;
    std::vector<std::string> limits;
    double adjustment;
    LimitPolicy adjustment_type = LimitPolicy::NONE;
}TRJ_INFO;

using trj_info_map = std::map<int, TRJ_INFO>;
static const std::map<std::string, trj_info_map> trj_type_map = {
    { "motor", {
        { iit::advr::Gains_Type_POSITION,  { "position", {"Min_pos", "Max_pos"},0.5f * M_PI / 180.0f, LimitPolicy::MARGIN} },
        { iit::advr::Gains_Type_VELOCITY,  { "velocity", {"Max_vel"},           0.95,LimitPolicy::SCALE } },
        { iit::advr::Gains_Type_IMPEDANCE, { "position", {"Min_pos", "Max_pos"},0.5f * M_PI / 180.0f, LimitPolicy::MARGIN} },
        { iit::advr::Gains_Type_TORQUE,    { "torque",   {"Max_tor"},           0.95,LimitPolicy::SCALE } },
        { iit::advr::Gains_Type_CURRENT,   { "current",  {"Max_ref"},           0.95,LimitPolicy::SCALE } }
    }},

    { "valve", {
        { iit::advr::Gains_Type_POSITION,  { "position", {}, 1.0,LimitPolicy::NONE } },
        { iit::advr::Gains_Type_IMPEDANCE, { "force", {},    1.0,LimitPolicy::NONE} },
        { iit::advr::Gains_Type_CURRENT,   { "current", {},  1.0,LimitPolicy::NONE } }
    }},

    { "pump", {
        { 0x39,                            { "pwm", {},     1.0,LimitPolicy::NONE } },
        { iit::advr::Gains_Type_VELOCITY,  { "velocity", {},1.0,LimitPolicy::NONE } },
        { iit::advr::Gains_Type_IMPEDANCE, { "pressure", {},1.0,LimitPolicy::NONE } }
    }}
};

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
