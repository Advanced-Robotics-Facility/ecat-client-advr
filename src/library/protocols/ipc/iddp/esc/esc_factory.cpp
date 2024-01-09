#include "esc_factory.h"

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
                            moto->init();
                            moto->write_connect();
                            _motors_iface_map[id] = moto;
                            _escs_iface_map[id] = std::static_pointer_cast<esc_pipe_iface >(moto);
                        }
                        break;
                    case FT6 :
                        {
                            auto ft = std::make_shared<ft6_iface>(robot_name, id);
                            ft->init();
                            ft->write_connect();
                            _fts_iface_map[id] = ft;
                            _escs_iface_map[id] = std::static_pointer_cast<esc_pipe_iface >(ft);
                        }
                        break;
                        
                    case IMU_ANY :
                        {
                            auto imu = std::make_shared<imu_iface>(robot_name, id);
                            imu->init();
                            imu->write_connect();
                            _imus_iface_map[id] = imu;
                            _escs_iface_map[id] = std::static_pointer_cast<esc_pipe_iface >(imu);
                        }
                        break;
                        
                    case POW_F28M36_BOARD :
                        {
                            auto pow = std::make_shared<powf28m36_iface>(robot_name, id);
                            pow->init();
                            pow->write_connect();
                            _pows_iface_map[id] = pow;
                            _escs_iface_map[id] = std::static_pointer_cast<esc_pipe_iface >(pow);
                        }
                        break;
                    
                    default:
                        throw(EscPipeIfaceError(EscPipeIfaceErrorNum::INVALID_TYPE, "Esc type NOT handled"));
                        break;
                }   
            }catch ( const EscPipeIfaceError &e) {
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
            
            std::vector<float> ft_v;
            
            ft_v.push_back(ft->force_x);
            ft_v.push_back(ft->force_y);
            ft_v.push_back(ft->force_z);
            
            ft_v.push_back(ft->torque_x);
            ft_v.push_back(ft->torque_y);
            ft_v.push_back(ft->torque_z);

            
            ft_status_map[id]= ft_v;

            
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
            
            std::vector<float> pow_v;

            pow_v.push_back(pow->v_batt);
            pow_v.push_back(pow->v_load);
            pow_v.push_back(pow->i_load);
            
            pow_v.push_back(pow->temp_batt);
            pow_v.push_back(pow->temp_heatsink);
            pow_v.push_back(pow->temp_pcb);
            
//             pow_v.push_back(pow->fault);
//             pow_v.push_back(pow->status);

            
            pow_status_map[id]= pow_v;

            
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
            
            std::vector<float> imu_v;

            imu_v.push_back(imu->x_rate);
            imu_v.push_back(imu->y_rate);
            imu_v.push_back(imu->z_rate);
            
            imu_v.push_back(imu->x_acc);
            imu_v.push_back(imu->y_acc);
            imu_v.push_back(imu->z_acc);
            
            imu_v.push_back(imu->x_quat);
            imu_v.push_back(imu->y_quat);
            imu_v.push_back(imu->z_quat);
            imu_v.push_back(imu->w_quat);

            
            imu_status_map[id]= imu_v;

            
        }
        catch ( std::out_of_range ) {};   
    }
}

