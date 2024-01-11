#ifndef EC_IDDP_H
#define EC_IDDP_H

#include <thread_util.h>
#include <pb_utils.h>
#include "esc_factory.h"

#include "ec_logger.h"
#include "ec_cmd.h"

#include <mutex>

class EcIDDP : public EcCmd,Thread_hook
{
public:

    EcIDDP(std::string host_address,uint32_t host_port);
    ~EcIDDP();

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
    SSI _slave_info;
    std::shared_ptr<EscFactory> _escs_factory;

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
    
    pthread_mutex_t _mutex_motor_status;
    pthread_mutex_t _mutex_motor_reference;
    pthread_mutex_t _mutex_ft6_status;
    pthread_mutex_t _mutex_pow_status;
    pthread_mutex_t _mutex_imu_status;
    
};

#endif // EC_IDDP_H
