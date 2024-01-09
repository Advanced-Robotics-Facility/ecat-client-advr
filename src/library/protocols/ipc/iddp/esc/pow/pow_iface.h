#ifndef __POW_IFACE__
#define __POW_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"

class powf28m36_iface: public esc_pipe_iface
{

public:

    powf28m36_iface(std::string robot_name,
                    int id);

    void get_from_pb(void);
    void set_to_pb(void);

    // rx_pdo values
    float v_batt, v_load, i_load;
    float temp_batt, temp_heatsink, temp_pcb;
    float fault, status;

};


#endif
