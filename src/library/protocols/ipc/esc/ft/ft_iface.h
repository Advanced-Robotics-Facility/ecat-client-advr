#ifndef __FT_IFACE__
#define __FT_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"

class ft6_iface: public esc_pipe_iface {

public:

    ft6_iface(std::string robot_name,
              int id);

    void get_from_pb(void);

    void set_to_pb(void);

    // rx_pdo values

    float force_x, force_y, force_z;
    
    float torque_x, torque_y, torque_z;
    
    float aux;
    
    uint32_t op_idx_ack, fault;

};


#endif
