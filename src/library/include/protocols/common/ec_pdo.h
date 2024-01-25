#ifndef __EC_PDO__
#define __EC_PDO__

#include "ec_types.h"
#include "protocols/common/esc/motor/motor_iface.h"
#include "protocols/common/esc/imu/imu_iface.h"
#include "protocols/common/esc/ft/ft_iface.h"
#include "protocols/common/esc/pow/pow_iface.h"

template <class T>
class EcPdo
{
    public:
        EcPdo(std::string protocol,std::string host_address,uint32_t host_port);
        EcPdo(std::string robot_name="None");
        ~EcPdo();
        
        void esc_factory(SSI slave_descr);
        void read_motor_pdo(MotorStatusMap &motor_status_map);
        void write_motor_pdo(const std::vector<MR> motors_references);
        void read_ft_pdo(FtStatusMap &ft_status_map);
        void read_imu_pdo(ImuStatusMap &imu_status_map);
        void read_pow_pdo(PwrStatusMap &pow_status_map);
        
    private:
        
        std::map<int, std::shared_ptr<MotorPdo<T>>> _moto_pdo_map;
        std::map<int, std::shared_ptr<FtPdo<T>>> _ft_pdo_map;
        std::map<int, std::shared_ptr<PowPdo<T>>> _pow_pdo_map;
        std::map<int, std::shared_ptr<ImuPdo<T>>> _imu_pdo_map;
        
        std::string _protocol;
        std::string _host_address;
        uint32_t _host_port;
        
        std::string _robot_name;
        std::string _ec_pdo_start;
        
};

#endif
