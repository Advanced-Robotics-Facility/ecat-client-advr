#include <cassert>
#include <tuple>

#include "ec_iddp.h"


EcIDDP::EcIDDP(std::string host_address,uint32_t host_port):
  EcCmd(host_address,host_port)
{
    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
     _mutex_motor_status= std::make_shared<std::mutex>();
    _mutex_motor_reference= std::make_shared<std::mutex>();

    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();

    _mutex_ft6_status= std::make_shared<std::mutex>();
    
    _mutex_pow_status= std::make_shared<std::mutex>();
    
    _mutex_imu_status= std::make_shared<std::mutex>();
    
}

EcIDDP::~EcIDDP()
{

}

//******************************* INIT *****************************************************//

void EcIDDP::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
    
    std::string robot_name = "NoNe";
    for ( auto &[id, esc_type, pos] : _slave_info ) {
        try { 
            // iface_factory will populate escs_iface map
            try { 
                _escs_iface[id] = iface_factory(id,esc_type,pos,robot_name); 
                
            }catch ( const EscPipeIfaceError &e) {
                DPRINTF("%s\n", e.what());
            }
    
        }catch(std::out_of_range) {
            DPRINTF("NOT mapped Esc id 0x%04X pos %d\n", id, pos );
        }
    }
}

void EcIDDP::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms,_logging);
}

void EcIDDP::start_client(uint32_t period_ms,bool logging)
{
    // periodic
    struct timespec ts;
    iit::ecat::us2ts(&ts, 1000*period_ms);
    // period.period is a timeval ... tv_usec 
    period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    _logging=logging;
    if(_logging)
    {
        start_logging();
    }
    
    retrieve_slaves_info(_slave_info);
    
    if(!_slave_info.empty()){
        create(true); // real time thread
        _client_alive=true;
    }
    
}

void EcIDDP::stop_client()
{
    stop();
    
    stop_logging();
}

bool EcIDDP::is_client_alive()
{
    return _client_alive;
}

void EcIDDP::start_logging()
{
    stop_logging();
    _ec_logger->start_mat_logger();
}

void EcIDDP::stop_logging()
{
    _ec_logger->stop_mat_logger();
}


//******************************* Periodic Activity *****************************************************//

void EcIDDP::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    
    // Receive motors, imu, ft, power board pdo information // 
    for (auto const &[id,esc_iface] : _escs_iface )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                if ( (nbytes = esc_iface->read()) > 0 ) {
                }
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
        }
        catch ( std::out_of_range ) {};   
    }

    // Send motors references
    _mutex_motor_reference->lock();

    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE &&
       !_motors_references.empty())
    {
        for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : _motors_references ) {
        
            auto _ctrl_type = static_cast<iit::advr::Gains_Type>(ctrl_type);
            std::shared_ptr<motor_iface> moto = std::static_pointer_cast<motor_iface >(_escs_iface[bId]);
            
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
    }
    _mutex_motor_reference->unlock();
}

void EcIDDP::periodicActivity()
{
    
        
}
//******************************* Periodic Activity *****************************************************//



void EcIDDP::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
{
    _mutex_motor_reference->lock();

    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();

    if(_client_alive)
    {
       if(motor_ref_flags==MotorRefFlags::FLAG_MULTI_REF ||
          motor_ref_flags==MotorRefFlags::FLAG_LAST_REF)
       {
           if(!motors_references.empty())
           {
                _motor_ref_flags = motor_ref_flags;
                _motors_references = motors_references;
                _ec_logger->log_set_motors_ref(_motors_references);
           }
           else
           {
                //_consoleLog->error("Motors references vector is empty!, please fill the vector");
           }
       }
       else
       {
           if(motor_ref_flags!=MotorRefFlags::FLAG_NONE)
           {
                //_consoleLog->error("Wrong motors references flag!");
           }
       }
    }
    else
    {
        //_consoleLog->error("UDP client not alive, please stop the main process");
    }

    _mutex_motor_reference->unlock();
}

MotorStatusMap EcIDDP::get_motors_status()
{
    _mutex_motor_status->lock();
    
    for (auto const &[id,esc_iface] : _motor_status_map)  {
        
        std::shared_ptr<motor_iface> moto = std::static_pointer_cast<motor_iface >(_escs_iface[id]);
        _motor_status_map[id] = std::make_tuple(moto->link_pos,moto->motor_pos,moto->link_vel,moto->motor_vel,
                                                moto->torque,moto->motor_temperature,moto->board_temperature,
                                                moto->fault,0,moto->aux_rd_idx_ack,moto->aux_rd,0);
    }
    
    _mutex_motor_status->unlock();
    
    return _motor_status_map;
}

FtStatusMap EcIDDP::get_ft6_status()
{
    _mutex_ft6_status->lock();
    
    auto ret_ft_status_map= _ft_status_map;
    
    _mutex_ft6_status->unlock();
    
    return ret_ft_status_map; 
}

PwrStatusMap EcIDDP::get_pow_status()
{
    _mutex_pow_status->lock();
    
    auto ret_pow_status_map= _pow_status_map;
    
    _mutex_pow_status->unlock();
    
    return ret_pow_status_map; 
}


ImuStatusMap EcIDDP::get_imu_status()
{
    _mutex_imu_status->lock();
    
    auto ret_imu_status_map= _imu_status_map;
    
    _mutex_imu_status->unlock();
    
    return _imu_status_map; 
}
bool EcIDDP::pdo_aux_cmd_sts(const PAC & pac)
{
    return false;
}
