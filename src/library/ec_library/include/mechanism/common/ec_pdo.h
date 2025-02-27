#ifndef __EC_PDO__
#define __EC_PDO__

#include "api/ec_iface.h"

template <class T>
class EcPdo: public virtual EcIface
{
    public:
        EcPdo(std::string protocol,std::string host_address,uint32_t host_port);
        EcPdo(std::string robot_name="None");
        ~EcPdo();
        
        void esc_factory(SSI slave_descr);
        bool init_read_pdo();
        void read_pdo();
        void write_pdo();
        void stop_pdo();
        
    private:
        template<typename MapPdo>
        void get_init_rx_pdo(const MapPdo& pdo_map);
        
        void read_motor_pdo();
        void write_motor_pdo();
        
        void read_ft_pdo();
        void read_imu_pdo();
        void read_pow_pdo();
        
        void read_valve_pdo();
        void write_valve_pdo();
        
        void read_pump_pdo();
        void write_pump_pdo();
        
        std::map<int, std::shared_ptr<MotorPdo<T>>> _moto_pdo_map;
        std::map<int, std::shared_ptr<FtPdo<T>>> _ft_pdo_map;
        std::map<int, std::shared_ptr<PowPdo<T>>> _pow_pdo_map;
        std::map<int, std::shared_ptr<ImuPdo<T>>> _imu_pdo_map;
        std::map<int, std::shared_ptr<ValvePdo<T>>> _valve_pdo_map;
        std::map<int, std::shared_ptr<PumpPdo<T>>> _pump_pdo_map;
        
        std::string _protocol;
        std::string _host_address;
        uint32_t _host_port;
        
        std::string _robot_name;
        std::string _ec_pdo_start;
        bool _init_read_pdo,_init_rx_pdo;
        int _id_init_err_read;
};

#endif
