#ifndef EC_LOGGER_H
#define EC_LOGGER_H

#include <memory>
#include <array>
#include <vector>
#include <map>

#include <matlogger2/matlogger2.h>
#include "api/ec_types.h"

class EcLogger
{
public:
    
    typedef std::shared_ptr<EcLogger> Ptr;
    
    EcLogger();
    
    void init_mat_logger(SSI slave_descr);
    void start_mat_logger();
    void stop_mat_logger();
    
    void log_motors_ref(const MotorReferenceMap& motors_ref);
    void log_motors_sts(const MotorStatusMap& motors_sts_map);
    void log_pow_sts(const PwrStatusMap& pow_sts_map);
    void log_ft_sts(const FtStatusMap& ft_sts_map);
    void log_imu_sts(const ImuStatusMap& imu_sts_map);
    void log_valve_sts(const ValveStatusMap& valve_sts_map);
    void log_valve_ref(const ValveReferenceMap& valves_ref);
    void log_pump_sts(const PumpStatusMap& pump_sts_map);
    void log_pump_ref(const PumpReferenceMap& pumps_ref);
    
    
private: 
    XBot::MatLogger2::Ptr _motors_references_logger;
    XBot::MatLogger2::Ptr _valves_references_logger;
    XBot::MatLogger2::Ptr _pumps_references_logger;
    XBot::MatLogger2::Ptr _motors_status_logger;
    XBot::MatLogger2::Ptr _ft_status_logger;
    XBot::MatLogger2::Ptr _pow_status_logger;
    XBot::MatLogger2::Ptr _imu_status_logger;
    XBot::MatLogger2::Ptr _valve_status_logger;
    XBot::MatLogger2::Ptr _pump_status_logger;
    
    std::vector<float> _motor_rx_v,_motor_tx_v;
    std::vector<float> _pow_rx_v;
    std::vector<float> _ft_rx_v;
    std::vector<float> _imu_rx_v;
    std::vector<float> _pump_rx_v,_pump_tx_v;
    std::vector<float> _valve_rx_v,_valve_tx_v;
    
    SSI _slave_descr;
    std::map<int,std::string> _log_motor_ref_map;
    std::map<int,std::string> _log_valve_ref_map;
    std::map<int,std::string> _log_pump_ref_map;
    std::map<int,std::string> _log_motor_sts_map;
    std::map<int,std::string> _log_ft_map;
    std::map<int,std::string> _log_imu_map;
    std::map<int,std::string> _log_pow_map;
    std::map<int,std::string> _log_valve_map;
    std::map<int,std::string> _log_pump_map;
};

#endif // EC_LOGGER_H
