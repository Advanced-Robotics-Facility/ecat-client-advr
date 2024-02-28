#ifndef EC_IFACE_H
#define EC_IFACE_H

#include <memory>
#include <array>
#include <vector>
#include <map>
#include "ec_types.h"
#include "logger/ec_logger.h"
#include "cmn_utils.h"

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
    void get_motors_status(MotorStatusMap &motor_status_map);
    void get_ft_status(FtStatusMap &ft_status_map);
    void get_pow_status(PwrStatusMap &pow_status_map);
    void get_imu_status(ImuStatusMap &imu_status_map);
    void get_valve_status(ValveStatusMap &valve_status_map);
    void get_pump_status(PumpStatusMap &pump_status_map);
    bool pdo_aux_cmd_sts(const PAC & pac);
    
    // EtherCAT Client ADVR Facilty setters
    void set_motors_references(const RefFlags, const std::vector<MR>);
    void set_valves_references(const RefFlags valve_ref_flags,const ValveReferenceMap valves_references);
    void set_pump_references(const RefFlags pump_ref_flags,const PumpReferenceMap pumps_references);
    
    // EtherCAT Client ADVR Facilty manager
    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;
    void test_client(SSI slave_info);
    
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
    std::shared_ptr<spdlog::logger> _consoleLog;
    
    bool _client_alive;
    bool _logging;
    ClientStatus _client_status;

    SSI _fake_slave_info;
    // last received motor data
    MotorStatusMap _motor_status_map;
    // last received ft data
    FtStatusMap _ft_status_map;
    // last received pow data
    PwrStatusMap _pow_status_map;
    // last received imu data
    ImuStatusMap _imu_status_map;
    // last received valve data
    ValveStatusMap _valve_status_map;
    // last received pump data
    PumpStatusMap _pump_status_map;
    
    RefFlags _motor_ref_flags;
    std::vector<MR> _motors_references;
    
    RefFlags _valve_ref_flags;
    ValveReferenceMap _valves_references;
    
    RefFlags _pump_ref_flags;
    PumpReferenceMap _pumps_references;
    
    pthread_mutex_t _mutex_motor_status;
    pthread_mutex_t _mutex_motor_reference;
    pthread_mutex_t _mutex_ft_status;
    pthread_mutex_t _mutex_pow_status;
    pthread_mutex_t _mutex_imu_status;
    pthread_mutex_t _mutex_valve_status;
    pthread_mutex_t _mutex_valve_reference;
    pthread_mutex_t _mutex_pump_status;
    pthread_mutex_t _mutex_pump_reference;
};

#endif // EC_IFACE_H
