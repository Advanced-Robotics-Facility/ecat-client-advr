#include "mechanism/common/ec_pdo.h"

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
    _init_read_pdo=_init_rx_pdo=false;

    if(_protocol!="pipe"){
        EcZmqPdoContext::start_context();
    }   
}
template < class T >
EcPdo<T>::EcPdo(std::string robot_name):
_robot_name(robot_name)
{
     _ec_pdo_start=_robot_name;
     _protocol="pipe";
    _init_read_pdo=_init_rx_pdo=false;
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
                    auto advrf_pdo = std::make_shared<AdvrfPdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(advrf_pdo);
                    _internal_motor_status_map[id]=_motor_status_map[id]=  advrf_pdo->rx_pdo;
                    _motor_reference_map[id]= advrf_pdo->tx_pdo;
                }break;
                case iit::ecat::SYNAPTICON_v5_0:
                case iit::ecat::SYNAPTICON_v5_1:{
                    auto synapticon_pdo = std::make_shared<SynapticonPdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(synapticon_pdo);
                    _internal_motor_status_map[id]=_motor_status_map[id]=  synapticon_pdo->rx_pdo;
                    _motor_reference_map[id]= synapticon_pdo->tx_pdo;
                }break;
                case iit::ecat::FT6_MSP432:{
                    auto ft_pdo = std::make_shared<FtPdo<T>>(_ec_pdo_start, id);
                    _ft_pdo_map[id]=ft_pdo;
                    _internal_ft_status_map[id]=_ft_status_map[id]= ft_pdo->rx_pdo;
                }break;   
                case iit::ecat::IMU_ANY :{
                    auto imu_pdo = std::make_shared<ImuPdo<T>>(_ec_pdo_start, id);
                    _imu_pdo_map[id]=imu_pdo;
                    _internal_imu_status_map[id]=_imu_status_map[id]= imu_pdo->rx_pdo;
                }break;
                case iit::ecat::POW_F28M36_BOARD :{
                    auto pow_pdo = std::make_shared<PowPdo<T>>(_ec_pdo_start, id);
                    _pow_pdo_map[id]=pow_pdo;
                    _internal_pow_status_map[id]=_pow_status_map[id]= pow_pdo->rx_pdo;
                }break;
                case iit::ecat::HYQ_KNEE:{
                    auto valve_pdo = std::make_shared<ValvePdo<T>>(_ec_pdo_start, id);
                    _valve_pdo_map[id]=valve_pdo;
                    _internal_valve_status_map[id]=_valve_status_map[id]=  valve_pdo->rx_pdo;
                    _valve_reference_map[id]= valve_pdo->tx_pdo;
                }break;
                case iit::ecat::HYQ_HPU:{
                    auto pump_pdo = std::make_shared<PumpPdo<T>>(_ec_pdo_start, id);
                    _pump_pdo_map[id]=pump_pdo;
                    _internal_pump_status_map[id]=_pump_status_map[id]= pump_pdo->rx_pdo;
                    _pump_reference_map[id]= pump_pdo->tx_pdo;
                }break;
                
                default:
                    break;
        }               
    }
} 

template < class T >
void EcPdo<T>::stop_pdo()
{
    _moto_pdo_map.clear();
    _ft_pdo_map.clear();
    _imu_pdo_map.clear();
    _pow_pdo_map.clear();
    _valve_pdo_map.clear();
    _pump_pdo_map.clear();
    
    if(_protocol!="pipe"){
        EcZmqPdoContext::stop_context();
    }
}

template < class T >
bool EcPdo<T>::init_read_pdo()
{
    struct timespec delay = { 0, 10000000UL }; //10ms
    int count=0;
    while(!_init_read_pdo && count<5){

        _init_rx_pdo=true; // start all bits from true
        read_pdo(); // read for 5 times.
        _init_read_pdo=_init_rx_pdo;

        if(!_init_read_pdo){
            count++;
            nanosleep(&delay, NULL);
        }

    }
    if(!_init_read_pdo){
        DPRINTF("Fatal Error on read PDO: Id [%d] is not initialized\n",_id_init_err_read);
    }
    else{
        DPRINTF("Success on init read pdo function!\n");
    }
    return true;
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
    if(_write_device[DeviceCtrlType::MOTOR]){
        write_motor_pdo();
        _write_device[DeviceCtrlType::MOTOR]=false;
    }
    
    if(_write_device[DeviceCtrlType::VALVE]){
        write_valve_pdo();
        _write_device[DeviceCtrlType::VALVE]=false;
    }

    if(_write_device[DeviceCtrlType::PUMP]){
        write_pump_pdo();
        _write_device[DeviceCtrlType::PUMP]=false;
    }
}

template <class T > 
template <typename MapPdo>
void EcPdo<T>::get_init_rx_pdo(const MapPdo& pdo_map)
{
    if(!_init_read_pdo){
        for (auto const &[id,pdo] : pdo_map ){
            if(!_init_rx_pdo){
                return;
            }
            _init_rx_pdo&= pdo->init_rx_pdo; // and all bits.
            if(!pdo->init_rx_pdo){
                _id_init_err_read=id;
            }
        }
    }
}

template < class T >
void EcPdo<T>::read_motor_pdo()
{
    for (auto const &[id,motor_pdo] : _moto_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0;
            do {
                // read protobuf data
                nbytes = motor_pdo->read();
            } while ( nbytes > 0);

            _internal_motor_status_map[id]=motor_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        
        catch ( const std::out_of_range &e) {};   
    }

    get_init_rx_pdo(_moto_pdo_map);
    if(!_internal_motor_status_map.empty()){
        _motor_status_queue.push(_internal_motor_status_map);
    }
}

template < class T >
void EcPdo<T>::write_motor_pdo()
{
    for (auto &[id,motor_pdo] : _moto_pdo_map ) {
        motor_pdo->tx_pdo=_motor_reference_map[id];

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

template < class T >
void EcPdo<T>::read_ft_pdo()
{
    for (auto const &[id,ft_pdo] : _ft_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0;
            do {
                // read protobuf data
                nbytes = ft_pdo->read();
            } while ( nbytes > 0);

            _internal_ft_status_map[id]=ft_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        catch ( const std::out_of_range &e) {};   
    }

    get_init_rx_pdo(_ft_pdo_map);
    if(!_internal_ft_status_map.empty()){
        _ft_status_queue.push(_internal_ft_status_map);
    }
}
template < class T >
void EcPdo<T>::read_imu_pdo()
{
    for (auto const &[id,imu_pdo] : _imu_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0;
            do {
                // read protobuf data
                nbytes = imu_pdo->read();
            } while ( nbytes > 0);

            _internal_imu_status_map[id]=imu_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        catch ( const std::out_of_range &e) {}; 
    }

    get_init_rx_pdo(_imu_pdo_map);
    if(!_internal_imu_status_map.empty()){
        _imu_status_queue.push(_internal_imu_status_map);
    }
}

template < class T >
void EcPdo<T>::read_pow_pdo()
{
    for (auto const &[id,pow_pdo] : _pow_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0;
            do {
                // read protobuf data
                nbytes = pow_pdo->read();
            } while ( nbytes > 0);

            _internal_pow_status_map[id]=pow_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        catch ( const std::out_of_range &e) {};     
    }

    get_init_rx_pdo(_pow_pdo_map);
    if(!_internal_pow_status_map.empty()){
        _pow_status_queue.push(_internal_pow_status_map);
    }
}

template < class T >
void EcPdo<T>::read_valve_pdo()
{
    for (auto const &[id,valve_pdo] : _valve_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0,count_read=0;
            do {
                // read protobuf data
                nbytes = valve_pdo->read();
                if(nbytes>0){
                    count_read++;
                }
            } while ( nbytes > 0);
            
            _internal_valve_status_map[id]=valve_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        catch ( const std::out_of_range &e) {};  
    }

    get_init_rx_pdo(_valve_pdo_map);
    if(!_internal_valve_status_map.empty()){
        _valve_status_queue.push(_internal_valve_status_map);
    }
}

template < class T >
void EcPdo<T>::write_valve_pdo()
{
    for (auto &[id,valve_pdo] : _valve_pdo_map ) {
        valve_pdo->tx_pdo=_valve_reference_map[id];
        //write 
        valve_pdo->write();
    }
}

template < class T >
void EcPdo<T>::read_pump_pdo()
{
    for (auto const &[id,pump_pdo] : _pump_pdo_map )  {
        try { 
            ///////////////////////////////////////////////////////////////
            // read
            int nbytes=0;
            do {
                // read protobuf data
                nbytes = pump_pdo->read();
            } while ( nbytes > 0);

            _internal_pump_status_map[id]=pump_pdo->rx_pdo;
            //////////////////////////////////////////////////////////////
        }
        catch ( const std::out_of_range &e) {};  
    }

    get_init_rx_pdo(_pump_pdo_map);
    if(!_internal_pump_status_map.empty()){
        _pump_status_queue.push(_internal_pump_status_map);
    }
}

template < class T >
void EcPdo<T>::write_pump_pdo()
{
    for (auto &[id,pump_pdo] : _pump_pdo_map)  {
        pump_pdo->tx_pdo=_pump_reference_map[id];
        //write 
        pump_pdo->write();
    }
}

template class EcPdo<EcPipePdo>;
template class EcPdo<EcZmqPdo>;
