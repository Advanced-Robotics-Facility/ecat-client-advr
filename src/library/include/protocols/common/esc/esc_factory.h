#ifndef __ESC_FACTORY__
#define __ESC_FACTORY__

#include "ec_types.h"
#include "protocols/common/esc/esc_iface.h"
#include "protocols/common/esc/motor/motor_iface.h"
#include "protocols/common/esc/imu/imu_iface.h"
#include "protocols/common/esc/ft/ft_iface.h"
#include "protocols/common/esc/pow/pow_iface.h"


class EscFactory
{
    public:
        EscFactory(SSI slave_descr,std::string robot_name);
        ~EscFactory();
        void read_motors(MotorStatusMap &motor_status_map);
        void feed_motors(const std::vector<MR> motors_references);
        
        void read_fts(FtStatusMap &ft_status_map);
        void read_imus(ImuStatusMap &imu_status_map);
        void read_pows(PwrStatusMap &pow_status_map);
        
    private:
        std::map<int, std::shared_ptr<esc_pipe_iface>>  _escs_iface_map;
        std::map<int, std::shared_ptr<motor_iface>>  _motors_iface_map;
        std::map<int, std::shared_ptr<ft6_iface>>  _fts_iface_map;
        std::map<int, std::shared_ptr<imu_iface>>  _imus_iface_map;
        std::map<int, std::shared_ptr<powf28m36_iface>>  _pows_iface_map;
};

#endif
