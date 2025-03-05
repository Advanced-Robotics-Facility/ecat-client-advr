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

#define MAX_QUEUE_PDO 64

using namespace boost::lockfree;
class EcIface
{
public:
    
    typedef std::shared_ptr<EcIface> Ptr;

    typedef struct CLIENT_STATUS_t{
        ClientStatusEnum status;
        bool run_loop;
    }CLIENT_STATUS;

    typedef struct CLIENT_THREAD_INFO_t{
        int cpu;
        int priority;
        int policy;
    }CLIENT_THREAD_INFO;
    
    EcIface();
    virtual ~EcIface();

    CLIENT_STATUS get_client_status(void);
    CLIENT_THREAD_INFO get_client_thread_info();
    
    // EtherCAT Client ADVR Facilty update getters/setters
    virtual void read(void);
    virtual void write(void);
    
    // EtherCAT Client ADVR Facilty getters
    void get_motor_status(MotorStatusMap &motor_status_map);
    void get_ft_status(FtStatusMap &ft_status_map);
    void get_pow_status(PwrStatusMap &pow_status_map);
    void get_imu_status(ImuStatusMap &imu_status_map);
    void get_valve_status(ValveStatusMap &valve_status_map);
    void get_pump_status(PumpStatusMap &pump_status_map);
    bool pdo_aux_cmd_sts(const PAC & pac);
    
    // EtherCAT Client ADVR Facilty setters
    void set_motor_reference(const MotorReferenceMap motor_reference);
    void set_valve_reference(const ValveReferenceMap valve_reference);
    void set_pump_reference(const PumpReferenceMap pump_reference);
    
    // EtherCAT Client ADVR Facilty manager
    virtual void start_client(uint32_t period_ms) = 0;
    virtual void stop_client(void) = 0;
    virtual void set_loop_time(uint32_t period_ms) = 0;
    
    // EtherCAT Client ADVR Facilty commands
    virtual bool start_devices(const DST &) = 0;
    virtual bool stop_devices() = 0;
    virtual bool pdo_aux_cmd(const PAC & pac) = 0;

    virtual bool retrieve_slaves_info(SSI &slave_info) = 0;
    void set_slaves_info(SSI slave_info);
    virtual bool retrieve_all_sdo(uint32_t esc_id,RR_SDOS &rr_sdo) = 0;
    virtual bool retrieve_rr_sdo(uint32_t esc_id,
                                 const RD_SDO &rd_sdo, 
                                 const WR_SDO &wr_sdo,
                                 RR_SDOS &rr_sdo) = 0;
    virtual bool set_wr_sdo(uint32_t esc_id,
                            const RD_SDO &rd_sdo,
                            const WR_SDO &wr_sdo) = 0;
protected:
    std::shared_ptr<spdlog::logger> _consoleLog;
    struct timespec _client_ts;

    uint64_t _period_ns;

    CLIENT_STATUS _client_status;
    CLIENT_THREAD_INFO _client_thread_info;

    SSI _fake_slave_info;
    // last received motor data
    MotorStatusMap _motor_status_map,_internal_motor_status_map;
    spsc_queue<MotorStatusMap,fixed_sized<true>> _motor_status_queue{MAX_QUEUE_PDO};

    // last received ft data
    FtStatusMap _ft_status_map,_internal_ft_status_map;
    spsc_queue<FtStatusMap,fixed_sized<true>> _ft_status_queue{MAX_QUEUE_PDO};
    // last received pow data
    PwrStatusMap _pow_status_map,_internal_pow_status_map;
    spsc_queue<PwrStatusMap,fixed_sized<true>> _pow_status_queue{MAX_QUEUE_PDO};
    // last received imu data
    ImuStatusMap _imu_status_map,_internal_imu_status_map;
    spsc_queue<ImuStatusMap,fixed_sized<true>> _imu_status_queue{MAX_QUEUE_PDO};
    // last received valve data
    ValveStatusMap _valve_status_map,_internal_valve_status_map;
    spsc_queue<ValveStatusMap,fixed_sized<true>> _valve_status_queue{MAX_QUEUE_PDO};
    // last received pump data
    PumpStatusMap _pump_status_map,_internal_pump_status_map;
    spsc_queue<PumpStatusMap,fixed_sized<true>> _pump_status_queue{MAX_QUEUE_PDO};
    
    MotorReferenceMap _motor_reference_map,_internal_motor_reference_map;
    spsc_queue<MotorReferenceMap,fixed_sized<true>> _motor_reference_queue{MAX_QUEUE_PDO};
    
    ValveReferenceMap _valve_reference_map,_internal_valve_reference_map;
    spsc_queue<ValveReferenceMap,fixed_sized<true>> _valve_reference_queue{MAX_QUEUE_PDO};
    
    PumpReferenceMap _pump_reference_map,_internal_pump_reference_map;
    spsc_queue<PumpReferenceMap,fixed_sized<true>> _pump_reference_queue{MAX_QUEUE_PDO};
    
    pthread_mutex_t _mutex_update,_mutex_client_thread;
    pthread_cond_t _update_cond,_client_thread_cond;
    pthread_condattr_t _update_attr;
    int _update_count=0;
    unsigned int _waiting_client_counter=0;
    
    void sync_client_thread();
    void wake_client_thread();
    
private:
    template <typename T>
    bool check_maps(const std::map<int32_t,T>& map1,const std::map<int32_t,T>& map2,std::string map_type);
};

#endif // EC_IFACE_H
