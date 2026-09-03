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
        
        if(ec_motors().count(esc_type)>0){
            switch ( esc_type ){
                case iit::ecat::CENTAC_v15 :
                case iit::ecat::CENTAC_v17 :
                case iit::ecat::LP:{
                    auto advrf_pdo = std::make_shared<AdvrfPdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(advrf_pdo);
                    _motor_status_map[id]=  advrf_pdo->rx_pdo;
                    _motor_reference_map[id]= advrf_pdo->tx_pdo;
                }break;
                default:{ //default cia402
                    auto cia402_pdo = std::make_shared<Cia402Pdo<T>>(_ec_pdo_start, id, esc_type);
                    _moto_pdo_map[id]=std::static_pointer_cast<MotorPdo<T>>(cia402_pdo);
                    _motor_status_map[id]=  cia402_pdo->rx_pdo;
                    _motor_reference_map[id]= cia402_pdo->tx_pdo;
                }break;
            }
        } else if(ec_valves().count(esc_type)>0){
            auto valve_pdo = std::make_shared<ValvePdo<T>>(_ec_pdo_start, id, esc_type);
            _valve_pdo_map[id]=valve_pdo;
            _valve_status_map[id]=  valve_pdo->rx_pdo;
            _valve_reference_map[id]= valve_pdo->tx_pdo;
        } else if(ec_pumps().count(esc_type)>0){
            auto pump_pdo = std::make_shared<PumpPdo<T>>(_ec_pdo_start, id, esc_type);
            _pump_pdo_map[id]=pump_pdo;
            _pump_status_map[id]= pump_pdo->rx_pdo;
            _pump_reference_map[id]= pump_pdo->tx_pdo;
        } else if(ec_grippers().count(esc_type)>0){
            auto gripper_pdo = std::make_shared<GripperPdo<T>>(_ec_pdo_start, id, esc_type);
            _gripper_pdo_map[id]=gripper_pdo;
            _gripper_status_map[id]= gripper_pdo->rx_pdo;
            _gripper_reference_map[id]= gripper_pdo->tx_pdo;
        } else{
            switch ( esc_type ){
                case iit::ecat::FT6MSP432_v24:{
                    auto ft_pdo = std::make_shared<FtPdo<T>>(_ec_pdo_start, id, esc_type);
                    _ft_pdo_map[id]=ft_pdo;
                    _ft_status_map[id]= ft_pdo->rx_pdo;
                }break;   
                case iit::ecat::IMUVN :{
                    auto imu_pdo = std::make_shared<ImuPdo<T>>(_ec_pdo_start, id, esc_type);
                    _imu_pdo_map[id]=imu_pdo;
                    _imu_status_map[id]= imu_pdo->rx_pdo;
                }break;
                case iit::ecat::POWF28M36 :{
                    auto pow_pdo = std::make_shared<PowPdo<T>>(_ec_pdo_start, id, esc_type);
                    _pow_pdo_map[id]=pow_pdo;
                    _pow_status_map[id]= pow_pdo->rx_pdo;
                }break;
                default:
                    break;
            }
        }               
    }

    _internal_motor_status.resize(_motor_status_map.size());
    _internal_valve_status.resize(_valve_status_map.size());
    _internal_pump_status.resize(_pump_status_map.size());
    _internal_gripper_status.resize(_gripper_status_map.size());

    _internal_ft_status.resize(_ft_status_map.size());
    _internal_imu_status.resize(_imu_status_map.size());
    _internal_pow_status.resize(_pow_status_map.size());
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
    _gripper_pdo_map.clear();
    
    if(_protocol!="pipe"){
        EcZmqPdoContext::stop_context();
    }
}

template < class T >
bool EcPdo<T>::init_read_pdo()
{
    struct timespec delay = { 0, 10000000UL }; //10ms
    int count=0;
    while(!_init_read_pdo && count<100){

        _init_rx_pdo=true; // start all bits from true
        read_pdo(); // read for 100 times.
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
    return _init_read_pdo;
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
void EcPdo<T>::read_pdo()
{
    const auto read_esc_pdo =
    [this](auto& pdo_map, auto& pdo_status,auto& queue) -> void
    {

        if(pdo_map.empty()){
            return;
        }

        std::size_t index = 0;

        for (auto const &[id,pdo] : pdo_map )  {
            try { 
                ///////////////////////////////////////////////////////////////
                // read
                int nbytes=0;
                do {
                    // read protobuf data
                    nbytes = pdo->read();
                } while ( nbytes > 0);

                pdo_status[index]=pdo->rx_pdo;
                //////////////////////////////////////////////////////////////
            }
            
            catch ( const std::out_of_range &e) {};  

            ++index;
        }

        get_init_rx_pdo(pdo_map);
        if(!pdo_status.empty()){
            queue.push(pdo_status);
        }

    };

    read_esc_pdo(_moto_pdo_map,_internal_motor_status,_motor_status_queue);
    read_esc_pdo(_ft_pdo_map,_internal_ft_status,_ft_status_queue);
    read_esc_pdo(_imu_pdo_map,_internal_imu_status,_imu_status_queue);
    read_esc_pdo(_pow_pdo_map,_internal_pow_status,_pow_status_queue);
    read_esc_pdo(_valve_pdo_map,_internal_valve_status,_valve_status_queue);
    read_esc_pdo(_pump_pdo_map,_internal_pump_status,_pump_status_queue);
    read_esc_pdo(_gripper_pdo_map,_internal_gripper_status,_gripper_status_queue);
}


template<int Index>
using CtrlIndex = std::integral_constant<int, Index>;

template < class T >
void EcPdo<T>::write_pdo()
{
    const auto write_esc_pdo =
    [this](const auto& dev_type,auto& pdo_map,const auto& ref_map,auto ctrl_index)
    {
        constexpr int index = decltype(ctrl_index)::value;

        if (!_write_device[dev_type]) {
            return;
        }

        for (auto& [id, pdo] : pdo_map) {
            pdo->tx_pdo = ref_map.at(id);

            if constexpr (index >= 0) {
                const auto ctrl_type = std::get<index>(pdo->tx_pdo);

                if (ctrl_type == 0x00) {
                    continue;
                }

                if (!iit::advr::Gains_Type_IsValid(ctrl_type)) {
                    DPRINTF("Control mode not recognized for id 0x%04X\n",id);
                    continue;
                }
            }

            pdo->write();
        }

        _write_device[dev_type] = false;
    };

    write_esc_pdo(DeviceCtrlType::MOTOR,   _moto_pdo_map,    _motor_reference_map,   CtrlIndex<0>{});   // motor ctrl mode--> index 0
    write_esc_pdo(DeviceCtrlType::VALVE,   _valve_pdo_map,   _valve_reference_map,   CtrlIndex<-1>{});  // no ctrl mode
    write_esc_pdo(DeviceCtrlType::PUMP,    _pump_pdo_map,    _pump_reference_map,    CtrlIndex<-1>{});  // no ctrl mode
    write_esc_pdo(DeviceCtrlType::GRIPPER, _gripper_pdo_map, _gripper_reference_map, CtrlIndex<-1>{});  // no ctrl mode
}

template class EcPdo<EcPipePdo>;
template class EcPdo<EcZmqPdo>;
