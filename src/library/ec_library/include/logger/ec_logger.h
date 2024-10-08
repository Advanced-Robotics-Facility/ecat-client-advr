#ifndef EC_LOGGER_H
#define EC_LOGGER_H

#include <memory>
#include <array>
#include <vector>
#include <map>

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
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
    void create_logger(std::string log_name,
                       int esc_id,
                       std::string log_esc_type,
                       int log_row);

    std::map<std::string,XBot::MatLogger2::Ptr> _log_map;
    XBot::MatAppender::Ptr _log_appender;
    XBot::MatLogger2::Options _log_opt;

    std::map<int,std::string> _log_stsEsc_map,_log_refEsc_map;
    std::map<int,std::vector<float>> _log_stsRow_map,_log_refRow_map;
    std::string _log_dir;
    SSI _slave_descr;
};

#endif // EC_LOGGER_H
