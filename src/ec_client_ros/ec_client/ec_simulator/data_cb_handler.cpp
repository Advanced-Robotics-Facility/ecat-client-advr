#include <stdint.h>

#include <utils.h>

#include "data_cb_handler.h"


static void motor_job(const SH_PIFACE esc_iface) {

    auto moto = std::dynamic_pointer_cast<motor_iface>(esc_iface);
    if ( moto ) {
        moto->pos_ref = moto->motor_pos;
        moto->vel_ref = 0;
        moto->tor_ref = 0;
        moto->info();
    }
            
}

/**
 * @brief 
 * 
 * @param id 
 * @param esc_iface 
 */
 void Default_pb_handle(const uint32_t id, const SH_PIFACE &esc_iface,bool read_only) {
    
    const std::string       name = "Default_pb_handle";

    uint32_t    esc_type = esc_iface->get_type();
    uint32_t    read_cnt = 0;
    int         nbytes;
    std::stringstream ss;
    
    ///////////////////////////////////////////////////////////////
    // read
    do {
        // read protobuf data
        if ( (nbytes = esc_iface->read()) > 0 ) {
            read_cnt++;
            ss << "[" << read_cnt << "]" << std::endl;
            ss << "read " << nbytes << " from " <<  esc_iface->get_name() << std::endl;
        }
    } while ( nbytes > 0);
    //////////////////////////////////////////////////////////////
    //
    switch (esc_type)
    {
    case iit::ecat::CENT_AC :
    case iit::ecat::MSP432_TEST :
        motor_job(esc_iface);
        break;
    
    default:
        break;
    }
    //////////////////////////////////////////////////////////////
    // write protobuf data
    if(!read_only)
    {
        nbytes = esc_iface->write();
        if ( nbytes > 0) {
            ss << "write " << nbytes << " to " << esc_iface->get_name() << std::endl;   
        }
    }
    
    printf ( "%s", ss.str().c_str() );

}

