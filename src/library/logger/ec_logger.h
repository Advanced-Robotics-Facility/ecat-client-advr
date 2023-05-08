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
    
    void start_mat_logger();
    void stop_mat_logger();
    void add_motors_ref(std::vector<MR> motors_ref,XBot::MatLogger2::Ptr logger);
    void log_motors_ref(std::vector<MR> motors_ref);
    void log_set_motors_ref(std::vector<MR> motors_ref);
    void log_motors_sts(MotorStatusMap motors_sts_map);
    void log_pow_sts(PwrStatusMap pow_sts_map);
    void log_ft6_sts(FtStatusMap ft_sts_map);
    
    void resize_motors_ref(size_t size);
    void resize_motors_sts(size_t size);
    void resize_pow_sts(size_t size);
    void resize_ft6_sts(size_t size);
    
private: 

    XBot::MatLogger2::Ptr _motors_references_logger;
    XBot::MatLogger2::Ptr _set_motors_references_logger;
    XBot::MatLogger2::Ptr _motors_status_logger;
    XBot::MatLogger2::Ptr _ft6_status_logger;
    XBot::MatLogger2::Ptr _pow_status_logger;
    
    Eigen::VectorXd _pos_ref_eigen,_vel_ref_eigen,_tor_ref_eigen,_GP_eigen,_GD_eigen;
    Eigen::VectorXd _link_pos_eigen,_motor_pos_eigen,_link_vel_eigen,_motor_vel_eigen,_tor_eigen,_motor_temp_eigen,_board_temp_eigen,_fault_eigen;
    Eigen::VectorXd _v_batt_eigen,_v_load_eigen,_i_load_eigen,_temp_pcb_eigen,_temp_heatsink_eigen,_temp_batt_eigen;
    Eigen::VectorXd _force_x_eigen,_force_y_eigen,_force_z_eigen,_torque_x_eigen,_torque_y_eigen,_torque_z_eigen;
                        


};

#endif // EC_LOGGER_H
