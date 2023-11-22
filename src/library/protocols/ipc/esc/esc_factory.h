#ifndef __ESC_FACTORY__
#define __ESC_FACTORY__

#include "ec_types.h"
#include "esc_iface.h"
#include "motor/motor_iface.h"
#include "imu/imu_iface.h"
#include "ft/ft_iface.h"
#include "pow/pow_iface.h"


using SH_PIFACE = std::shared_ptr<esc_pipe_iface>;

inline SH_PIFACE iface_factory(int id, uint32_t esc_type, int pos, const std::string robot_name)
{
    SH_PIFACE esc_iface;

    switch ( esc_type  )
    {
        case CENT_AC :
            // scope to avoid cross initialization of variable moto
            {
                auto moto = std::make_shared<motor_iface>(robot_name, id, esc_type);
                moto->init();
                esc_iface = std::static_pointer_cast<esc_pipe_iface >(moto);
            }
            break;
        
        default:
            throw(EscPipeIfaceError(EscPipeIfaceErrorNum::INVALID_TYPE, "Esc type NOT handled"));
            break;
    } 

    return esc_iface;       
} 


#endif
