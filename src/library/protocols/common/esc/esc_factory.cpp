#include "protocols/common/esc/esc_factory.h"

EscFactory::EscFactory(SSI slave_descr,std::string robot_name)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        try { 
            // iface_factory will populate escs_iface map
            try { 
                switch ( esc_type  )
                {
                    case CENT_AC:
                    case LO_PWR_DC_MC :
                        {
                            auto moto = std::make_shared<motor_iface>(robot_name, id, esc_type);
                            _motors_iface_map[id] = moto;
                            _escs_iface_map[id] = std::static_pointer_cast<EcPipePdo >(moto);
                        }
                        break;
                    case FT6 :
                        {
                            auto ft = std::make_shared<ft6_iface>(robot_name, id);
                            _fts_iface_map[id] = ft;
                            _escs_iface_map[id] = std::static_pointer_cast<EcPipePdo >(ft);
                        }
                        break;
                        
                    case IMU_ANY :
                        {
                            auto imu = std::make_shared<imu_iface>(robot_name, id);
                            _imus_iface_map[id] = imu;
                            _escs_iface_map[id] = std::static_pointer_cast<EcPipePdo >(imu);
                        }
                        break;
                        
                    case POW_F28M36_BOARD :
                        {
                            auto pow = std::make_shared<powf28m36_iface>(robot_name, id);
                            _pows_iface_map[id] = pow;
                            _escs_iface_map[id] = std::static_pointer_cast<EcPipePdo >(pow);
                        }
                        break;
                    
                    default:
                        throw(EcPipePdoError(EcPipePdoErrorNum::INVALID_TYPE, "Esc type NOT handled"));
                        break;
                }   
            }catch ( const EcPipePdoError &e) {
                DPRINTF("%s\n", e.what());
            }
    
        }catch(std::out_of_range) {
            DPRINTF("NOT mapped Esc id 0x%04X pos %d\n", id, pos );
        }
    }
} 

EscFactory::~EscFactory()
{
    for (auto const &[id,esc_iface] : _escs_iface_map )  {
        esc_iface->write_quit();
    }

    for ( auto& [id, esc_iface] : _escs_iface_map ) {
        esc_iface.reset();
    }
}

void EscFactory::read_motors(MotorStatusMap &motor_status_map)
{
    for (auto const &[id,moto] : _motors_iface_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = moto->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            
            motor_status_map[id] = std::make_tuple(moto->link_pos,moto->motor_pos,moto->link_vel,moto->motor_vel,
                                                   moto->torque,moto->motor_temperature,moto->board_temperature,
                                                   moto->fault,moto->rtt,moto->aux_rd_idx_ack,moto->aux_rd,moto->cmd_aux_sts);
            
        }
        catch ( std::out_of_range ) {};   
    }
}

void EscFactory::feed_motors(const std::vector<MR> motors_references)
{
    for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : motors_references ) {
        auto _ctrl_type = static_cast<iit::advr::Gains_Type>(ctrl_type);
        if(_motors_iface_map.count(bId) > 0){
            auto moto = _motors_iface_map[bId];
            
            moto->pos_ref= pos;
            moto->vel_ref= vel;
            moto->tor_ref= tor;
            
            if ( (_ctrl_type == iit::advr::Gains_Type_POSITION ||
            _ctrl_type == iit::advr::Gains_Type_VELOCITY)) {
                moto->kp_ref= g0;
                moto->kd_ref= g2;
                moto->tau_p_ref=0;
                moto->tau_fc_ref=0;
                moto->tau_d_ref=0;
            }
            else if ( _ctrl_type == iit::advr::Gains_Type_IMPEDANCE) {
                moto->kp_ref= g0;
                moto->kd_ref= g1;
                moto->tau_p_ref=g2;
                moto->tau_fc_ref=g3;
                moto->tau_d_ref=g4;
            } else {

            }
            auto _op = static_cast<iit::advr::AuxPDO_Op>(op);

            switch (_op)
            {
                case iit::advr::AuxPDO_Op_SET:
                    moto->aux_wr_idx=idx;
                    moto->aux_wr=aux;
                    break;
                case iit::advr::AuxPDO_Op_GET:
                    moto->aux_rd_idx_req=idx;
                    break;
                case iit::advr::AuxPDO_Op_NOP:
                    break;
            }
            
            //write 
            moto->write();
        }
        else{
            DPRINTF("Cannot send reference to id 0x%04X \n", bId);
        }
    }
}

void EscFactory::read_fts(FtStatusMap &ft_status_map)
{
    for (auto const &[id,ft] : _fts_iface_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = ft->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            ft_status_map[id]= ft->ft_v; 
        }
        catch ( std::out_of_range ) {};   
    }
}

void EscFactory::read_pows(PwrStatusMap &pow_status_map)
{
    for (auto const &[id,pow] : _pows_iface_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = pow->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            pow_status_map[id]= pow->pow_v;
        }
        catch ( std::out_of_range ) {};   
    }
}

void EscFactory::read_imus(ImuStatusMap &imu_status_map)
{
    for (auto const &[id,imu] : _imus_iface_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = imu->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            imu_status_map[id]= imu->imu_v;
        }
        catch ( std::out_of_range ) {};   
    }
}

