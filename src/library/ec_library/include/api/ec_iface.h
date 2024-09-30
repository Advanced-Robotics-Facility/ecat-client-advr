#ifndef EC_IFACE_H
#define EC_IFACE_H

#include <memory>
#include <array>
#include <vector>
#include <map>
#include "ec_types.h"
#include "logger/ec_logger.h"
#include "cmn_utils.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/stack.hpp>

using namespace boost::lockfree;
class EcIface
{
public:
    
    typedef std::shared_ptr<EcIface> Ptr;

    typedef struct CLIENT_STATUS_t{
        ClientStatusEnum status;
        bool run_loop;
        std::vector<bool> devices_started;
    }CLIENT_STATUS;
    
    EcIface();
    virtual ~EcIface();

    CLIENT_STATUS get_client_status(void);
    
    // EtherCAT Client ADVR Facilty logger
    void start_logging(void);
    void stop_logging(void);
    void log(void);

    // EtherCAT Client ADVR Facilty update getters/setters
    void read(void);
    void write(void);
    
    // EtherCAT Client ADVR Facilty getters
    void get_motors_status(MotorStatusMap &motor_status_map);
    void get_ft_status(FtStatusMap &ft_status_map);
    void get_pow_status(PwrStatusMap &pow_status_map);
    void get_imu_status(ImuStatusMap &imu_status_map);
    void get_valve_status(ValveStatusMap &valve_status_map);
    void get_pump_status(PumpStatusMap &pump_status_map);
    bool pdo_aux_cmd_sts(const PAC & pac);
    
    // EtherCAT Client ADVR Facilty setters
    void set_motors_references(const RefFlags motor_ref_flags,const MotorReferenceMap motors_references);
    void set_valves_references(const RefFlags valve_ref_flags,const ValveReferenceMap valves_references);
    void set_pumps_references(const RefFlags pump_ref_flags,const PumpReferenceMap pumps_references);
    
    // EtherCAT Client ADVR Facilty manager
    virtual void start_client(uint32_t period_ms,bool logging) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;
    
    // EtherCAT Client ADVR Facilty commands
    virtual bool start_motors(const MST &) = 0;
    virtual bool stop_motors(void) = 0;
    virtual bool pdo_aux_cmd(const PAC & pac) = 0;

    virtual bool retrieve_slaves_info(SSI &slave_info) = 0;
    void set_slaves_info(SSI slave_info);
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
    struct timespec _client_ts;
    
    bool _logging;
    uint64_t _period_ns;

    CLIENT_STATUS _client_status;

    SSI _fake_slave_info;
    // last received motor data
    MotorStatusMap _motor_status_map,_internal_motor_status_map;
    spsc_queue<MotorStatusMap,fixed_sized<true>> _motor_status_queue{128};

    // last received ft data
    FtStatusMap _ft_status_map,_internal_ft_status_map;
    spsc_queue<FtStatusMap,fixed_sized<true>> _ft_status_queue{128};
    // last received pow data
    PwrStatusMap _pow_status_map,_internal_pow_status_map;
    spsc_queue<PwrStatusMap,fixed_sized<true>> _pow_status_queue{128};
    // last received imu data
    ImuStatusMap _imu_status_map,_internal_imu_status_map;
    spsc_queue<ImuStatusMap,fixed_sized<true>> _imu_status_queue{128};
    // last received valve data
    ValveStatusMap _valve_status_map,_internal_valve_status_map;
    spsc_queue<ValveStatusMap,fixed_sized<true>> _valve_status_queue{128};
    // last received pump data
    PumpStatusMap _pump_status_map,_internal_pump_status_map;
    spsc_queue<PumpStatusMap,fixed_sized<true>> _pump_status_queue{128};
    
    MotorReferenceMap _motors_references,_internal_motors_references;
    spsc_queue<MotorReferenceMap,fixed_sized<true>> _motors_references_queue{128};
    
    ValveReferenceMap _valves_references,_internal_valves_references;
    spsc_queue<ValveReferenceMap,fixed_sized<true>> _valves_references_queue{128};
    
    PumpReferenceMap _pumps_references,_internal_pumps_references;
    spsc_queue<PumpReferenceMap,fixed_sized<true>> _pumps_references_queue{128};
    
    pthread_mutex_t _mutex_update,_mutex_client_thread;
    pthread_cond_t _update_cond,_client_thread_cond;
    pthread_condattr_t _update_attr;
    int _update_count=0;
    unsigned int _waiting_client_counter=0;
    
    void sync_client_thread();
    void wake_client_thread();
    bool all_devices_stopped();

private:
    template <typename T>
    int32_t check_maps(const std::map<int32_t,T>& map1,const std::map<int32_t,T>& map2);
};

#endif // EC_IFACE_H
