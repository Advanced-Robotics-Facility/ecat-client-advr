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
EcPdo<T>::EcPdo(std::string robot_name):
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
                case iit::ecat::CENT_AC :
                case iit::ecat::LO_PWR_DC_MC:{
                        auto hhcm_pdo = std::make_shared<HhcmPdo<T>>(_ec_pdo_start, id, esc_type);
                        _hhcm_pdo_map[id]=hhcm_pdo;
                        _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(hhcm_pdo);
                        _motor_status_map[id] = hhcm_pdo->mt_t;
                        _motors_references.push_back(std::make_tuple(id, 0x00,0,0,0,0,0,0,0,0,0,0,0));
                }break;
                case iit::ecat::CIRCULO9:{
                    auto circulo9_pdo = std::make_shared<Circulo9Pdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(circulo9_pdo);
                    _motor_status_map[id] = circulo9_pdo->mt_t;
                    _motors_references.push_back(std::make_tuple(id, 0x00,0,0,0,0,0,0,0,0,0,0,0));
                }break;
                case iit::ecat::AMC_FLEXPRO:{
                    auto flex_pdo = std::make_shared<FlexproPdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(flex_pdo);
                    _motor_status_map[id] = flex_pdo->mt_t;
                    _motors_references.push_back(std::make_tuple(id, 0x00,0,0,0,0,0,0,0,0,0,0,0));
                }break;
                case iit::ecat::FT6_MSP432:{
                    auto ft_pdo = std::make_shared<FtPdo<T>>(_ec_pdo_start, id);
                    _ft_pdo_map[id]=ft_pdo;
                    _ft_status_map[id]= ft_pdo->ft_v;
                }break;   
                case iit::ecat::IMU_ANY :{
                    auto imu_pdo = std::make_shared<ImuPdo<T>>(_ec_pdo_start, id);
                    _imu_pdo_map[id]=imu_pdo;
                    _imu_status_map[id]= imu_pdo->imu_v;
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                        auto pow_pdo = std::make_shared<PowPdo<T>>(_ec_pdo_start, id);
                        _pow_pdo_map[id]=pow_pdo;
                        _pow_status_map[id]= pow_pdo->pow_v;
                }break;
                case iit::ecat::HYQ_KNEE:{
                        auto valve_pdo = std::make_shared<ValvePdo<T>>(_ec_pdo_start, id);
                        _valve_pdo_map[id]=valve_pdo;
                        _valve_status_map[id]= valve_pdo->valve_v;
                        _valves_references.push_back(std::make_tuple(id, 0.0,0,0,0));
                }break;
                case iit::ecat::HYQ_HPU:{
                        auto pump_pdo = std::make_shared<PumpPdo<T>>(_ec_pdo_start, id);
                        _pump_pdo_map[id]=pump_pdo;
                        _pump_status_map[id]= pump_pdo->rx_pdo;
                }break;
                
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
    
    read_valve_pdo();
    
    read_pump_pdo();
}

template < class T >
void EcPdo<T>::write_pdo()
{
    write_motor_pdo();
    
    write_valve_pdo();
    
    write_pump_pdo();
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
            
            _motor_status_map[id] = motor_pdo->mt_t;
        }
        catch ( std::out_of_range ) {};   
    }
    _ec_logger->log_motors_sts(_motor_status_map);
    pthread_mutex_unlock(&_mutex_motor_status);
}

template < class T >
void EcPdo<T>::write_motor_pdo()
{
    pthread_mutex_lock(&_mutex_motor_reference);
    if(_motor_ref_flags!=RefFlags::FLAG_NONE){
        for ( const auto &[bId,ctrl_type,pos,vel,tor,g0,g1,g2,g3,g4,op,idx,aux] : _motors_references ) {
            if(_moto_pdo_map.count(bId) > 0 && ctrl_type!=0x00){
                
                auto ctrl_type_cast = static_cast<iit::advr::Gains_Type>(ctrl_type);
                if (iit::advr::Gains_Type_IsValid(ctrl_type) ) {
                    auto motor_pdo = _moto_pdo_map[bId];
                    
                    motor_pdo->tx_pdo.pos_ref= pos;
                    motor_pdo->tx_pdo.vel_ref= vel;
                    motor_pdo->tx_pdo.tor_ref= tor;
                    
                    motor_pdo->tx_pdo.gain_0= g0;
                    motor_pdo->tx_pdo.gain_1= g1;
                    motor_pdo->tx_pdo.gain_2= g2;
                    motor_pdo->tx_pdo.gain_3= g3;
                    motor_pdo->tx_pdo.gain_4= g4;
                    
                    if(_hhcm_pdo_map.count(bId)>0){
                        if ( (ctrl_type_cast == iit::advr::Gains_Type_POSITION ||
                              ctrl_type_cast == iit::advr::Gains_Type_VELOCITY)) {
                            motor_pdo->tx_pdo.gain_0= g0;
                            motor_pdo->tx_pdo.gain_1= g2;
                            motor_pdo->tx_pdo.gain_2= 0;
                            motor_pdo->tx_pdo.gain_3= 0;
                            motor_pdo->tx_pdo.gain_4= g1;
                        }
                    }
                
                    auto _op = static_cast<iit::advr::AuxPDO_Op>(op);

                    switch (_op)
                    {
                        case iit::advr::AuxPDO_Op_SET:
                            motor_pdo->tx_pdo.op_idx_aux=idx;
                            motor_pdo->tx_pdo.aux=aux;
                            break;
                        case iit::advr::AuxPDO_Op_GET:
                            motor_pdo->tx_pdo.op_idx_aux=idx;
                            break;
                        case iit::advr::AuxPDO_Op_NOP:
                            break;
                    }
                    
                    //write 
                    motor_pdo->write();
                }
                else{
                    DPRINTF("Control mode not recognized for id 0x%04X \n", bId);
                }
            }
            else{
#ifndef TEST_LIBRARY 
                DPRINTF("Cannot send motor reference to id 0x%04X \n", bId);
#endif                   
            }
        }
        _ec_logger->log_motors_ref(_motors_references);
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
            _ft_status_map[id]= ft_pdo->ft_v; 
        }
        catch ( std::out_of_range ) {};   
    }
    _ec_logger->log_ft_sts(_ft_status_map);
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
            _imu_status_map[id]= imu_pdo->imu_v;
        }
        catch ( std::out_of_range ) {};   
    }
    _ec_logger->log_imu_sts(_imu_status_map);
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
            _pow_status_map[id]= pow_pdo->pow_v;
        }
        catch ( std::out_of_range ) {};   
    }
    _ec_logger->log_pow_sts(_pow_status_map);
    pthread_mutex_unlock(&_mutex_pow_status);
}

template < class T >
void EcPdo<T>::read_valve_pdo()
{
    pthread_mutex_lock(&_mutex_valve_status);
    for (auto const &[id,valve_pdo] : _valve_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = valve_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            _valve_status_map[id]= valve_pdo->valve_v;
        }
        catch ( std::out_of_range ) {};   
    }
    _ec_logger->log_valve_sts(_valve_status_map);
    pthread_mutex_unlock(&_mutex_valve_status);
}
template < class T >
void EcPdo<T>::write_valve_pdo()
{
    pthread_mutex_lock(&_mutex_valve_reference);
    if(_valve_ref_flags!=RefFlags::FLAG_NONE){
        for ( const auto &[bId,curr_ref,pwm_1,pwm_2,dout] : _valves_references ) {
            if(_valve_pdo_map.count(bId) > 0 ){
                auto valve_pdo=_valve_pdo_map[bId];
                valve_pdo->tx_pdo.current_ref=curr_ref;
                valve_pdo->tx_pdo.pwm_1=pwm_1;
                valve_pdo->tx_pdo.pwm_2=pwm_2;
                valve_pdo->tx_pdo.dout=dout;
                valve_pdo->write();
            }
            else{
#ifndef TEST_LIBRARY 
                DPRINTF("Cannot send valve reference to id 0x%04X \n", bId);
#endif                   
            }
        }
        _ec_logger->log_valve_ref(_valves_references);
    }
    
    pthread_mutex_unlock(&_mutex_valve_reference);
}

template < class T >
void EcPdo<T>::read_pump_pdo()
{
    pthread_mutex_lock(&_mutex_pump_status);
    for (auto const &[id,pump_pdo] : _pump_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes;
            do {
                // read protobuf data
                nbytes = pump_pdo->read();
            } while ( nbytes > 0);
            //////////////////////////////////////////////////////////////
            _pump_status_map[id]= pump_pdo->rx_pdo;
        }
        catch ( std::out_of_range ) {};   
    }
    //_ec_logger->log_pow_sts(_pow_status_map);
    pthread_mutex_unlock(&_mutex_pump_status);
}
template < class T >
void EcPdo<T>::write_pump_pdo()
{
    pthread_mutex_lock(&_mutex_pump_reference);
    
    pthread_mutex_unlock(&_mutex_pump_reference);
}

template class EcPdo<EcPipePdo>;
template class EcPdo<EcZmqPdo>;
