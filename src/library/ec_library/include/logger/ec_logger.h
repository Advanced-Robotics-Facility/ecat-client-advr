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

    typedef struct LOGGER_INFO_t{
        XBot::MatLogger2::Ptr logger;
        std::map<int,std::string> logger_entry;
        std::map<int,std::vector<float>> logger_row;
    }LOGGER_INFO;
    
    typedef std::shared_ptr<EcLogger> Ptr;
    
    EcLogger(bool compression_enabled=true);
    
    void init_mat_logger(SSI slave_descr);
    void start_mat_logger();
    void stop_mat_logger();
    
    void log_motor_status(const MotorStatusMap& motor_status_map);
    void log_motor_reference(const MotorReferenceMap& motor_reference_map);
    void log_pow_status(const PwrStatusMap& pow_status_map);
    void log_ft_status(const FtStatusMap& ft_status_map);
    void log_imu_status(const ImuStatusMap& imu_status_map);
    void log_valve_status(const ValveStatusMap& valve_status_map);
    void log_valve_reference(const ValveReferenceMap& valve_reference_map);
    void log_pump_status(const PumpStatusMap& pump_status_map);
    void log_pump_reference(const PumpReferenceMap& pump_reference_map);
    
private: 
    void create_logger(std::string logger_name,
                       int esc_id,
                       std::string logger_entry_type,
                       int logger_row);

    std::string _logger_dir;
    XBot::MatLogger2::Options _logger_opt;
    std::map<std::string,std::shared_ptr<EcLogger::LOGGER_INFO>> _logger_map;
    XBot::MatAppender::Ptr _appender;

    SSI _slave_descr;
};

#endif // EC_LOGGER_H
