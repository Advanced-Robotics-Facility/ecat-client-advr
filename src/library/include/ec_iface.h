#ifndef EC_IFACE_H
#define EC_IFACE_H

#include <memory>
#include <array>
#include <vector>
#include <map>
#include "ec_types.h"
#include "logger/ec_logger.h"

class EcIface
{
public:
    
    typedef std::shared_ptr<EcIface> Ptr;
    
    EcIface();
    virtual ~EcIface();

    bool is_client_alive(void);
    
    // EtherCAT Client ADVR Facilty logger
    void start_logging(void);
    void stop_logging(void);
    
    // EtherCAT Client ADVR Facilty getters
    MotorStatusMap get_motors_status(void);
    FtStatusMap get_ft6_status(void);
    PwrStatusMap get_pow_status(void);
    ImuStatusMap get_imu_status(void);
    
    // EtherCAT Client ADVR Facilty setters
    void set_motors_references(const MotorRefFlags, const std::vector<MR>);
    
    
    virtual bool pdo_aux_cmd_sts(const PAC & pac) = 0;
    
    // EtherCAT Client ADVR Facilty manager
    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;
    
    // EtherCAT Client ADVR Facilty commands
    virtual bool start_motors(const MST &) = 0;
    virtual bool stop_motors(void) = 0;
    virtual bool pdo_aux_cmd(const PAC & pac) = 0;

    virtual bool retrieve_slaves_info(SSI &slave_info) = 0;
    virtual bool retrieve_all_sdo(uint32_t esc_id,RR_SDO &rr_sdo) = 0;
    virtual bool retrieve_rr_sdo(uint32_t esc_id,
                                 const RD_SDO &rd_sdo, 
                                 const WR_SDO &wr_sdo,
                                 RR_SDO &rr_sdo) = 0;
    virtual bool set_wr_sdo(uint32_t esc_id,
                            const RD_SDO &rd_sdo,
                            const WR_SDO &wr_sdo) = 0;
protected:
    EcLogger::Ptr _ec_logger;
    bool _client_alive;
    bool _logging;
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
    
    pthread_mutex_t _mutex_motor_status;
    pthread_mutex_t _mutex_motor_reference;
    pthread_mutex_t _mutex_ft6_status;
    pthread_mutex_t _mutex_pow_status;
    pthread_mutex_t _mutex_imu_status;
};

#endif // EC_IFACE_H
