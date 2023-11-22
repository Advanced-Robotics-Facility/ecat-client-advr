#ifndef EC_TCP_H
#define EC_TCP_H

#include <thread_util.h>
#include "ec_logger.h"
#include "ec_cmd.h"

#include <mutex>

class EcTCP : public EcCmd,Thread_hook
{
public:

    EcTCP(std::string host_address,uint32_t host_port);
    ~EcTCP();

    void start_client(uint32_t period_ms,bool logging) final;
    void stop_client() final ;
    bool is_client_alive() final;
    void set_loop_time(uint32_t period_ms) final;
    
    void receive_error(std::error_code ec);
    void periodicActivity();

    void start_logging() final;
    void stop_logging() final;
    
    void set_motors_references(const MotorRefFlags, const std::vector<MR>) final;
    
    MotorStatusMap get_motors_status() final ;
    FtStatusMap get_ft6_status() final;
    PwrStatusMap get_pow_status() final ;
    ImuStatusMap get_imu_status() final;
    bool pdo_aux_cmd_sts(const PAC & pac) final;

public:
    
    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
    
private:
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    
private:
    EcLogger::Ptr _ec_logger;
    bool _client_alive;
    bool _logging;
    std::shared_ptr<spdlog::logger> _consoleLog;
    SSI _slave_info;
    
    // last received motor data
    MotorStatusMap _motor_status_map;
    // last received ft data
    FtStatusMap _ft_status_map;
    // last received pow data
    PwrStatusMap _pow_status_map;
    // last received imu data
    ImuStatusMap _imu_status_map;
    
    MotorRefFlags _motor_ref_flags;
    std::vector<MR> _motors_references;
    
    std::shared_ptr<std::mutex> _mutex_motor_status;
    std::shared_ptr<std::mutex> _mutex_motor_reference;
    std::shared_ptr<std::mutex> _mutex_ft6_status;
    std::shared_ptr<std::mutex> _mutex_pow_status;
    std::shared_ptr<std::mutex> _mutex_imu_status;

};

#endif // EC_TCP_H
