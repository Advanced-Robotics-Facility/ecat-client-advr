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
    void start_mat_logger();
    void stop_mat_logger();
    
    void add_motors_ref(std::vector<MR> motors_ref,XBot::MatLogger2::Ptr logger);
    void log_motors_ref(std::vector<MR> motors_ref);
    void log_set_motors_ref(std::vector<MR> motors_ref);
    
    void log_motors_sts(const MotorStatusMap motors_sts_map);
    void log_pow_sts(const PwrStatusMap pow_sts_map);
    void log_ft_sts(const FtStatusMap ft_sts_map);
    void log_imu_sts(const ImuStatusMap imu_sts_map);
    
private: 

    XBot::MatLogger2::Ptr _motors_references_logger;
    XBot::MatLogger2::Ptr _set_motors_references_logger;
    XBot::MatLogger2::Ptr _motors_status_logger;
    XBot::MatLogger2::Ptr _ft_status_logger;
    XBot::MatLogger2::Ptr _pow_status_logger;
    XBot::MatLogger2::Ptr _imu_status_logger;
    
    Eigen::VectorXd _motor_ref_eigen,_motor_sts_eigen;
};

#endif // EC_LOGGER_H
