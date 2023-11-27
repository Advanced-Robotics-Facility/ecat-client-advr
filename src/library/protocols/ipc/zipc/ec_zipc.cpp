#include <cassert>
#include <tuple>

#include "ec_zipc.h"


EcZipc::EcZipc(std::string host_address,uint32_t host_port):
  EcCmd("ipc",host_address,host_port)
{
    if(host_address=="localhost")
    {
        host_address.clear();
        host_address="127.0.0.1";
    }
    _host_address=host_address;
    _host_port=host_port;
    
    _ec_logger = std::make_shared<EcLogger>();
    _logging=false;
    
     _mutex_motor_status= std::make_shared<std::mutex>();
    _mutex_motor_reference= std::make_shared<std::mutex>();

    _motor_ref_flags=MotorRefFlags::FLAG_NONE;
    _motors_references.clear();

    _mutex_ft6_status= std::make_shared<std::mutex>();
    
    _mutex_pow_status= std::make_shared<std::mutex>();
    
    _mutex_imu_status= std::make_shared<std::mutex>();
    
    _consoleLog=spdlog::get("console");
    if(!_consoleLog)
    {
        createLogger("console","client");
        _consoleLog=spdlog::get("console");
    }
}

EcZipc::~EcZipc()
{

}

//******************************* INIT *****************************************************//

void EcZipc::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
    
    for ( auto &[id, esc_type, pos] : _slave_info ) {
    
            std::string host_port_cmd = std::to_string(_host_port+id);
            // zmq setup
            std::string zmq_uri = "ipc://" + _host_address + ":"+host_port_cmd;
            EcZmqPdo::Ptr zmq_pdo = std::make_shared<EcZmqPdo>(zmq_uri);
            
            switch ( esc_type  )
            {
                    case CENT_AC:
                    case LO_PWR_DC_MC :
                        {
                            _moto_pdo_map[id]=zmq_pdo;
                        }
                        break;
                    case FT6 :
                        {
                            _ft_pdo_map[id]=zmq_pdo;
                        }
                        break;
                        
                    case IMU_ANY :
                        {
                             _imu_pdo_map[id]=zmq_pdo;
                        }
                        break;
                        
                    case POW_F28M36_BOARD :
                        {
                            _pow_pdo_map[id]=zmq_pdo;
                        }
                        break;
                    
                    default:
                        _consoleLog->error("Esc type NOT handled");
                        break;
            }               
    }
}

void EcZipc::set_loop_time(uint32_t period_ms)
{
   stop_client();
   
   start_client(period_ms,_logging);
}

void EcZipc::start_client(uint32_t period_ms,bool logging)
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
        create(false); // non-real time thread
        _client_alive=true;
    }
    
}

void EcZipc::stop_client()
{
    stop();
    
    stop_logging();
}

bool EcZipc::is_client_alive()
{
    return _client_alive;
}

void EcZipc::start_logging()
{
    stop_logging();
    _ec_logger->start_mat_logger();
}

void EcZipc::stop_logging()
{
    _ec_logger->stop_mat_logger();
}

void EcZipc::read_motors(MotorStatusMap &motor_status_map)
{
    for ( auto &[id, moto_pdo] : _moto_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(moto_pdo->zmq_recv_pdo(msg,pb_rx_pdos))
        {
            if(msg!="")
            {
                motor_status_map[id] = std::make_tuple(pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_pos(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_pos(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->link_vel(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_vel(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->torque(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->motor_temp(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->board_temp(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->fault(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->rtt(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->op_idx_ack(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->aux(),
                                                       pb_rx_pdos.mutable_motor_xt_rx_pdo()->cmd_aux_sts());
            }
        }
        else
        {
            _consoleLog->error("Error on Force/Torque reading on device: {}",id);
        }
    }
}
void EcZipc::read_fts(FtStatusMap &ft_status_map)
{
    for ( auto &[id, ft_pdo] : _ft_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(ft_pdo->zmq_recv_pdo(msg,pb_rx_pdos))
        {
            if(msg!="")
            {
                std::vector<float> ft_v;
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_x());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_z());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->force_y());
                
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_x());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_y());
                ft_v.push_back(pb_rx_pdos.mutable_ft6_rx_pdo()->torque_z());
                
                ft_status_map[id]=ft_v;
            }
        }
        else
        {
            _consoleLog->error("Error on Force/Torque reading on device: {}",id);
        }
    }
}
void EcZipc::read_imus(ImuStatusMap &imu_status_map)
{
    for ( auto &[id, imu_pdo] : _imu_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(imu_pdo->zmq_recv_pdo(msg,pb_rx_pdos))
        {
            if(msg!="")
            {
                std::vector<float> imu_v;
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_rate());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_rate());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_rate());
                
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_acc());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_acc());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_acc());
                
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->x_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->y_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->z_quat());
                imu_v.push_back(pb_rx_pdos.mutable_imuvn_rx_pdo()->w_quat());
                imu_status_map[id]=imu_v;
            }
        }
        else
        {
            _consoleLog->error("Error on IMU reading on device: {}",id);
        }
    }
}

void EcZipc::read_pows(PwrStatusMap &pow_status_map)
{
    for ( auto &[id, pow_pdo] : _pow_pdo_map ) {
        iit::advr::Ec_slave_pdo pb_rx_pdos;
        std::string msg="";
        if(pow_pdo->zmq_recv_pdo(msg,pb_rx_pdos))
        {
            if(msg!="")
            {
                std::vector<float> pow_v;
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_batt());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->v_load());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->i_load());
                
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_batt());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_heatsink());
                pow_v.push_back(pb_rx_pdos.mutable_powf28m36_rx_pdo()->temp_pcb());
                
                pow_status_map[id]=pow_v;
            }
        }
        else
        {
            _consoleLog->error("Error on Power Board reading on device: {}",id);
        }
    }
}


//******************************* Periodic Activity *****************************************************//

void EcZipc::th_loop( void * )
{
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    // Receive motors, imu, ft, power board pdo information // 
    _mutex_motor_status->lock();
    read_motors(_motor_status_map);
    _ec_logger->log_motors_sts(_motor_status_map);
    _mutex_motor_status->unlock();
    
    _mutex_ft6_status->lock();
    read_fts(_ft_status_map);
    _ec_logger->log_ft6_sts(_ft_status_map);
    _mutex_ft6_status->unlock();
    
    _mutex_imu_status->lock();
    read_imus(_imu_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
    _mutex_imu_status->unlock();
    
    _mutex_pow_status->lock();
    read_pows(_pow_status_map);
    _ec_logger->log_pow_sts(_pow_status_map);
    _mutex_pow_status->unlock();

    // Send motors references
    _mutex_motor_reference->lock();

    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE &&
        !_motors_references.empty())
    {
        feed_motors(_motors_references);
    }

    _mutex_motor_reference->unlock();
}

void EcZipc::periodicActivity()
{
    
        
}
//******************************* Periodic Activity *****************************************************//



void EcZipc::set_motors_references(const MotorRefFlags motor_ref_flags,const std::vector<MR> motors_references)
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
                _consoleLog->error("Motors references vector is empty!, please fill the vector");
           }
       }
       else
       {
           if(motor_ref_flags!=MotorRefFlags::FLAG_NONE)
           {
                _consoleLog->error("Wrong motors references flag!");
           }
       }
    }
    else
    {
        _consoleLog->error("Client not alive, please stop the main process");
    }

    _mutex_motor_reference->unlock();
}

MotorStatusMap EcZipc::get_motors_status()
{
    _mutex_motor_status->lock();
    
    auto ret_motor_status_map= _motor_status_map;
    
    _mutex_motor_status->unlock();
    
    return ret_motor_status_map;
}

FtStatusMap EcZipc::get_ft6_status()
{
    _mutex_ft6_status->lock();
    
    auto ret_ft_status_map= _ft_status_map;
    
    _mutex_ft6_status->unlock();
    
    return ret_ft_status_map; 
}

PwrStatusMap EcZipc::get_pow_status()
{
    _mutex_pow_status->lock();
    
    auto ret_pow_status_map= _pow_status_map;
    
    _mutex_pow_status->unlock();
    
    return ret_pow_status_map; 
}


ImuStatusMap EcZipc::get_imu_status()
{
    _mutex_imu_status->lock();
    
    auto ret_imu_status_map= _imu_status_map;
    
    _mutex_imu_status->unlock();
    
    return _imu_status_map; 
}
bool EcZipc::pdo_aux_cmd_sts(const PAC & pac)
{
    return false;
}
