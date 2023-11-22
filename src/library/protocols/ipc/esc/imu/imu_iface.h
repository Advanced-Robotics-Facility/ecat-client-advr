#ifndef __IMU_IFACE__
#define __IMU_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"

class imu_iface : public esc_pipe_iface
{

public:

    imu_iface(std::string robot_name,
              int id);

    void get_from_pb(void);

    void set_to_pb(void);

    // rx_pdo values
    float x_rate, y_rate, z_rate;
    float x_acc, y_acc, z_acc;
    float x_quat, y_quat, z_quat, w_quat;
    uint32_t imu_ts, temperature, digital_in, fault;


};


#endif
