#ifndef __EC_PDO__
#define __EC_PDO__

#include "ec_iface.h"
#include "ec_types.h"

#include "protocols/common/pdo/motor/motor_pdo.h"
#include "protocols/common/pdo/imu/imu_pdo.h"
#include "protocols/common/pdo/ft/ft_pdo.h"
#include "protocols/common/pdo/pow/pow_pdo.h"

template <class T>
class EcPdo: public virtual EcIface
{
    public:
        EcPdo(std::string protocol,std::string host_address,uint32_t host_port);
        EcPdo(std::string robot_name="None",uint32_t host_port=5000);
        ~EcPdo();
        
        void esc_factory(SSI slave_descr);
        void read_pdo();
        void write_pdo();
        
    private:
        
        void read_motor_pdo();
        void write_motor_pdo();
        void read_ft_pdo();
        void read_imu_pdo();
        void read_pow_pdo();
        
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
