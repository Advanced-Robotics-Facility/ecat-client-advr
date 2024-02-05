#include "common/ec_pdo.h"


template < class T >
EcPdo<T>::EcPdo(std::string protocol, std::string host_address, uint32_t host_port):
_protocol(protocol),
_host_address(host_address),
_host_port(host_port)
{
    _host_address=host_address;
    if(_host_address=="localhost")
    {
        _host_address.clear();
        _host_address="127.0.0.1";
    }
    
    _host_port=host_port+4000; // to be verified in the configuration file
    
}
template < class T >
EcPdo<T>::EcPdo(std::string robot_name,uint32_t host_port):
_robot_name(robot_name)
{
     _ec_pdo_start=_robot_name;
     _protocol="pipe";
}

template < class T >
EcPdo<T>::~EcPdo()
{
    
}

template < class T >
void EcPdo<T>::esc_factory(SSI slave_descr)
{
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        
        if(_protocol!="pipe"){
            std::string host_port_cmd = std::to_string(_host_port+id);
            // zmq setup
            std::string zmq_uri = _protocol+"://" + _host_address + ":"+host_port_cmd;
            _ec_pdo_start=zmq_uri;
        }
        
        switch ( esc_type  )
        {
                case CENT_AC:
                case LO_PWR_DC_MC :
                    {
                        
                        auto moto_pdo = std::make_shared<MotorPdo<T>>(_ec_pdo_start, id, esc_type);
                        _moto_pdo_map[id]=moto_pdo;
                    }
                    break;
                case FT6 :
                    {
                        auto ft_pdo = std::make_shared<FtPdo<T>>(_ec_pdo_start, id);
                        _ft_pdo_map[id]=ft_pdo;
                    }
                    break;
                    
                case IMU_ANY :
                    {
                        auto imu_pdo = std::make_shared<ImuPdo<T>>(_ec_pdo_start, id);
                        _imu_pdo_map[id]=imu_pdo;
                    }
                    break;
                    
                case POW_F28M36_BOARD :
                    {
                        auto pow_pdo = std::make_shared<PowPdo<T>>(_ec_pdo_start, id);
                        _pow_pdo_map[id]=pow_pdo;
                    }
                    break;
                
                default:
                    break;
        }               
    }
} 

template < class T >
void EcPdo<T>::read_pdo()
{
    read_motor_pdo();
    
    read_ft_pdo();
    
    read_imu_pdo();
    
    read_pow_pdo();
}

template < class T >
void EcPdo<T>::write_pdo()
{
    write_motor_pdo();
}



template < class T >
void EcPdo<T>::read_motor_pdo()
{
    pthread_mutex_lock(&_mutex_motor_status);
    for (auto const &[id,motor_pdo] : _moto_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = motor_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            
            _motor_status_map[id] = motor_pdo->_motor_pdo.mt_t;
        }
        catch ( std::out_of_range ) {};   
    }
    pthread_mutex_unlock(&_mutex_motor_status);
}

template < class T >
void EcPdo<T>::write_motor_pdo()
{
    pthread_mutex_lock(&_mutex_motor_reference);
    if(_motor_ref_flags!=MotorRefFlags::FLAG_NONE){
        for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : _motors_references ) {
            auto _ctrl_type = static_cast<iit::advr::Gains_Type>(ctrl_type);
            if(_moto_pdo_map.count(bId) > 0){
                auto motor_pdo = _moto_pdo_map[bId];
                
                motor_pdo->_motor_pdo.pos_ref= pos;
                motor_pdo->_motor_pdo.vel_ref= vel;
                motor_pdo->_motor_pdo.tor_ref= tor;
                
                if ( (_ctrl_type == iit::advr::Gains_Type_POSITION ||
                _ctrl_type == iit::advr::Gains_Type_VELOCITY)) {
                    motor_pdo->_motor_pdo.kp_ref= g0;
                    motor_pdo->_motor_pdo.kd_ref= g2;
                    motor_pdo->_motor_pdo.tau_p_ref=0;
                    motor_pdo->_motor_pdo.tau_fc_ref=0;
                    motor_pdo->_motor_pdo.tau_d_ref=0;
                }
                else if ( _ctrl_type == iit::advr::Gains_Type_IMPEDANCE) {
                    motor_pdo->_motor_pdo.kp_ref= g0;
                    motor_pdo->_motor_pdo.kd_ref= g1;
                    motor_pdo->_motor_pdo.tau_p_ref=g2;
                    motor_pdo->_motor_pdo.tau_fc_ref=g3;
                    motor_pdo->_motor_pdo.tau_d_ref=g4;
                } else {

                }
                auto _op = static_cast<iit::advr::AuxPDO_Op>(op);

                switch (_op)
                {
                    case iit::advr::AuxPDO_Op_SET:
                        motor_pdo->_motor_pdo.aux_wr_idx=idx;
                        motor_pdo->_motor_pdo.aux_wr=aux;
                        break;
                    case iit::advr::AuxPDO_Op_GET:
                        motor_pdo->_motor_pdo.aux_rd_idx_req=idx;
                        break;
                    case iit::advr::AuxPDO_Op_NOP:
                        break;
                }
                
                //write 
                motor_pdo->write();
            }
            else{
                //DPRINTF("Cannot send reference to id 0x%04X \n", bId);
            }
        }
        if(!_motors_references.empty()){
            _ec_logger->log_motors_ref(_motors_references);
        }
    }
    
    pthread_mutex_unlock(&_mutex_motor_reference);
}

template < class T >
void EcPdo<T>::read_ft_pdo()
{
    pthread_mutex_lock(&_mutex_ft_status);
    for (auto const &[id,ft_pdo] : _ft_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = ft_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            _ft_status_map[id]= ft_pdo->_ft_pdo.ft_v; 
        }
        catch ( std::out_of_range ) {};   
    }
    pthread_mutex_unlock(&_mutex_ft_status);
}
template < class T >
void EcPdo<T>::read_imu_pdo()
{
    pthread_mutex_lock(&_mutex_imu_status);
    for (auto const &[id,imu_pdo] : _imu_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = imu_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            _imu_status_map[id]= imu_pdo->_imu_pdo.imu_v;
        }
        catch ( std::out_of_range ) {};   
    }
    pthread_mutex_unlock(&_mutex_imu_status);
}

template < class T >
void EcPdo<T>::read_pow_pdo()
{
    pthread_mutex_lock(&_mutex_pow_status);
    for (auto const &[id,pow_pdo] : _pow_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = pow_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            _pow_status_map[id]= pow_pdo->_pow_pdo.pow_v;
        }
        catch ( std::out_of_range ) {};   
    }
    pthread_mutex_unlock(&_mutex_pow_status);
}

template class EcPdo<EcPipePdo>;
template class EcPdo<EcZmqPdo>;
