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
    _ec_pdo_start="";
    
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
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(hhcm_pdo);
                    _motor_status_map[id]=  hhcm_pdo->rx_pdo;
                    _motors_references[id]= hhcm_pdo->tx_pdo;
                }break;
                case iit::ecat::CIRCULO9:{
                    auto circulo9_pdo = std::make_shared<Circulo9Pdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(circulo9_pdo);
                    _motor_status_map[id]=  circulo9_pdo->rx_pdo;
                    _motors_references[id]= circulo9_pdo->tx_pdo;
                }break;
                case iit::ecat::AMC_FLEXPRO:{
                    auto flex_pdo = std::make_shared<FlexproPdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(flex_pdo);
                    _motor_status_map[id] = flex_pdo->rx_pdo;
                    _motors_references[id]= flex_pdo->tx_pdo;
                }break;
                case iit::ecat::FT6_MSP432:{
                    auto ft_pdo = std::make_shared<FtPdo<T>>(_ec_pdo_start, id);
                    _ft_pdo_map[id]=ft_pdo;
                    _ft_status_map[id]= ft_pdo->rx_pdo;
                }break;   
                case iit::ecat::IMU_ANY :{
                    auto imu_pdo = std::make_shared<ImuPdo<T>>(_ec_pdo_start, id);
                    _imu_pdo_map[id]=imu_pdo;
                    _imu_status_map[id]= imu_pdo->rx_pdo;
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                    auto pow_pdo = std::make_shared<PowPdo<T>>(_ec_pdo_start, id);
                    _pow_pdo_map[id]=pow_pdo;
                    _pow_status_map[id]= pow_pdo->rx_pdo;
                }break;
                case iit::ecat::HYQ_KNEE:{
                    auto valve_pdo = std::make_shared<ValvePdo<T>>(_ec_pdo_start, id);
                    _valve_pdo_map[id]=valve_pdo;
                    _valve_status_map[id]=  valve_pdo->rx_pdo;
                    _valves_references[id]= valve_pdo->tx_pdo;
                }break;
                case iit::ecat::HYQ_HPU:{
                    auto pump_pdo = std::make_shared<PumpPdo<T>>(_ec_pdo_start, id);
                    _pump_pdo_map[id]=pump_pdo;
                    _pump_status_map[id]= pump_pdo->rx_pdo;
                    _pumps_references[id]= pump_pdo->tx_pdo;
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


template <class T > 
template <typename MapPdo,typename MapStatus>
void EcPdo<T>::set_map_status(pthread_mutex_t &mutex_status,const MapPdo& pdo_map,MapStatus& map_status)
{
    pthread_mutex_lock(&mutex_status);
    for (auto const &[id,pdo] : pdo_map )  {
        map_status[id]=pdo->rx_pdo;
    }
    pthread_mutex_unlock(&mutex_status);
}

template <class T > 
template <typename MapReference,typename MapPdo>
void EcPdo<T>::get_map_reference(const MapReference& map_reference,
                                 MapPdo& pdo_map)
{
    for ( const auto &[id,tx_pdo] : map_reference ) {
        pdo_map[id]->tx_pdo=tx_pdo;
    }
}

template < class T >
void EcPdo<T>::read_motor_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_motor_status,_moto_pdo_map,_motor_status_map);
    _ec_logger->log_motors_sts(_motor_status_map);
}

template < class T >
void EcPdo<T>::write_motor_pdo()
{
    if(_motor_ref_flags!=RefFlags::FLAG_NONE){

        pthread_mutex_lock(&_mutex_motor_reference);
        _ec_logger->log_motors_ref(_motors_references);
        get_map_reference(_motors_references,_moto_pdo_map);
        pthread_mutex_unlock(&_mutex_motor_reference);

        for ( const auto &[id,motor_pdo] : _moto_pdo_map ) {
            auto ctrl_type=std::get<0>(motor_pdo->tx_pdo);
            if(ctrl_type!=0x00){
                if (iit::advr::Gains_Type_IsValid(ctrl_type) ) {
                    //write 
                    motor_pdo->write();
                }
                else{
                    DPRINTF("Control mode not recognized for id 0x%04X \n", id);
                }
            }
        }
    }
}

template < class T >
void EcPdo<T>::read_ft_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_ft_status,_ft_pdo_map,_ft_status_map);
    _ec_logger->log_ft_sts(_ft_status_map);
}
template < class T >
void EcPdo<T>::read_imu_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_imu_status,_imu_pdo_map,_imu_status_map);
    _ec_logger->log_imu_sts(_imu_status_map);
}

template < class T >
void EcPdo<T>::read_pow_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_pow_status,_pow_pdo_map,_pow_status_map);
    _ec_logger->log_pow_sts(_pow_status_map);
}

template < class T >
void EcPdo<T>::read_valve_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_valve_status,_valve_pdo_map,_valve_status_map);
    _ec_logger->log_valve_sts(_valve_status_map);
}

template < class T >
void EcPdo<T>::write_valve_pdo()
{
    if(_valve_ref_flags!=RefFlags::FLAG_NONE){


        pthread_mutex_lock(&_mutex_valve_reference);
        _ec_logger->log_valve_ref(_valves_references);
        get_map_reference(_valves_references,_valve_pdo_map);
        pthread_mutex_unlock(&_mutex_valve_reference);

        for ( const auto &[bId,valve_pdo] : _valve_pdo_map ) {
            //write 
            valve_pdo->write();
        }
    }
}

template < class T >
void EcPdo<T>::read_pump_pdo()
{
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
        }
        catch ( std::out_of_range ) {};   
    }

    set_map_status(_mutex_pump_status,_pump_pdo_map,_pump_status_map);
    _ec_logger->log_pump_sts(_pump_status_map);
}

template < class T >
void EcPdo<T>::write_pump_pdo()
{
    if(_pump_ref_flags!=RefFlags::FLAG_NONE){

        pthread_mutex_lock(&_mutex_pump_reference);
        _ec_logger->log_pump_ref(_pumps_references);
        get_map_reference(_pumps_references,_pump_pdo_map);
        pthread_mutex_unlock(&_mutex_pump_reference);        
        
        for (auto const &[id,pump_pdo] : _pump_pdo_map)  {
            //write 
            pump_pdo->write();
        }
    }
}

template class EcPdo<EcPipePdo>;
template class EcPdo<EcZmqPdo>;
