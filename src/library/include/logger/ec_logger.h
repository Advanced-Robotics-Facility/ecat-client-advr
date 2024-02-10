#ifndef EC_LOGGER_H
#define EC_LOGGER_H

#include <memory>
#include <array>
#include <vector>
#include <map>

#include <matlogger2/matlogger2.h>
#include "ec_types.h"

class EcLogger
{
public:
    
    typedef std::shared_ptr<EcLogger> Ptr;
    
    EcLogger();
    
    void init_mat_logger(SSI slave_descr);
    void start_mat_logger();
    void stop_mat_logger();
    
    void log_motors_ref(const std::vector<MR> motors_ref);
    void log_set_motors_ref(const std::vector<MR> motors_ref);
    void log_motors_sts(const MotorStatusMap motors_sts_map);
    void log_pow_sts(const PwrStatusMap pow_sts_map);
    void log_ft_sts(const FtStatusMap ft_sts_map);
    void log_imu_sts(const ImuStatusMap imu_sts_map);
    
private: 

    void add_motors_ref(const std::vector<MR> motors_ref,XBot::MatLogger2::Ptr logger);

    XBot::MatLogger2::Ptr _motors_references_logger;
    XBot::MatLogger2::Ptr _set_motors_references_logger;
    XBot::MatLogger2::Ptr _motors_status_logger;
    XBot::MatLogger2::Ptr _ft_status_logger;
    XBot::MatLogger2::Ptr _pow_status_logger;
    XBot::MatLogger2::Ptr _imu_status_logger;
    
    Eigen::VectorXd _motor_ref_eigen,_motor_sts_eigen;
    SSI _slave_descr;
    std::map<int,std::string> _log_motor_ref_map;
    std::map<int,std::string> _log_motor_sts_map;
    std::map<int,std::string> _log_ft_map;
    std::map<int,std::string> _log_imu_map;
    std::map<int,std::string> _log_pow_map;
};

#endif // EC_LOGGER_H
