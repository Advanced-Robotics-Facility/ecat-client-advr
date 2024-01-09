#ifndef __MOTOR_IFACE__
#define __MOTOR_IFACE__

#include <pb_utils.h>
#include "../esc_iface.h"


/**
 * @brief The motor_iface class implements the generic
 * esc_iface to handle an esc of type motor
 */
class motor_iface: public esc_pipe_iface {

public:

    /**
     * @brief motor_iface constructor
     * @param robot_name according to ecat master convention
     * @param id is the slave id
     */
    motor_iface( int32_t id, uint32_t type,
                 std::string rd_pp_name,
                 std::string wr_pp_name );

    motor_iface( const std::string ,int32_t id, uint32_t type);
    
    motor_iface( const std::string ,int32_t id);

    void get_from_pb(void) override;

    void set_to_pb(void) override;

    // tx_pdo values
    float pos_ref, vel_ref, tor_ref, kp_ref, kd_ref;
    float tau_p_ref, tau_d_ref, tau_fc_ref;
    uint32_t aux_rd_idx_req, aux_wr_idx;
    float aux_wr;

    // rx_pdo values
    float   motor_pos, motor_vel, link_pos, link_vel, torque;
    float   motor_temperature, board_temperature;
    uint32_t aux_rd_idx_ack, fault,rtt,cmd_aux_sts;
    float aux_rd;
    float read_pos_ref, read_vel_ref, read_torque_ref;  // ack from master


        
        
};


#endif
