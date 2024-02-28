#include "logger/ec_logger.h"

EcLogger::EcLogger()
{
    _motor_ref_eigen.resize(11);
    _motor_sts_eigen.resize(12);
    _pump_ref_eigen.resize(25);
    _pump_rx_v.resize(PumpPdoRx::pdo_size);
    _pump_tx_v.resize(PumpPdoTx::pdo_size);
    _valve_rx_v.resize(ValvePdoRx::pdo_size);
    _valve_tx_v.resize(ValvePdoTx::pdo_size);
}

void EcLogger::init_mat_logger(SSI slave_descr)
{
    _slave_descr.clear();
    _slave_descr=slave_descr;
}
void EcLogger::start_mat_logger()
{
    // Logger setup
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e4; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    
    for ( auto &[esc_id, esc_type, pos] : _slave_descr ) {
        switch ( esc_type  )
        {
            case iit::ecat::CENT_AC :
            case iit::ecat::LO_PWR_DC_MC :
            case iit::ecat::CIRCULO9:
            case iit::ecat::AMC_FLEXPRO:{
                if(_motors_status_logger==nullptr){
                    _motors_status_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_status_logger", opt);
                    _motors_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_motor_sts_map.count(esc_id)==0){
                    std::string motor_sts_id="motor_sts_id_"+std::to_string(esc_id);
                    std::cout << "mot_sts_id: " << motor_sts_id << std::endl;
                    _log_motor_sts_map[esc_id]=motor_sts_id;
                    _motors_status_logger->create(motor_sts_id,_motor_sts_eigen.size());
                }
                
                if(_motors_references_logger==nullptr){
                    _motors_references_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_references_logger");
                    _motors_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_motor_ref_map.count(esc_id)==0){
                    std::string motor_ref_id="motor_ref_id_"+std::to_string(esc_id);
                    _log_motor_ref_map[esc_id]=motor_ref_id;
                    _motors_references_logger->create(motor_ref_id,_motor_ref_eigen.size());
                }
            }break;
            case iit::ecat::FT6_MSP432:{
                if(_ft_status_logger==nullptr){
                    _ft_status_logger = XBot::MatLogger2::MakeLogger("/tmp/ft_status_logger", opt);
                    _ft_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                if(_log_ft_map.count(esc_id)==0){
                    std::string ft_id="ft_id_"+std::to_string(esc_id);
                    _log_ft_map[esc_id]=ft_id;
                    _ft_status_logger->create(ft_id,6);
                }
            }break; 
            case iit::ecat::IMU_ANY :{
                if(_imu_status_logger==nullptr){
                    _imu_status_logger = XBot::MatLogger2::MakeLogger("/tmp/imu_status_logger", opt);
                    _imu_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                if(_log_imu_map.count(esc_id)==0){
                    std::string imu_id="imu_id_"+std::to_string(esc_id);
                    _log_imu_map[esc_id]=imu_id;
                    _imu_status_logger->create(imu_id,10);
                }
            }break;
            case iit::ecat::POW_F28M36_BOARD :{
                if(_pow_status_logger==nullptr){
                    _pow_status_logger = XBot::MatLogger2::MakeLogger("/tmp/pow_status_logger", opt);
                    _pow_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                if(_log_pow_map.count(esc_id)==0){
                    std::string pow_id="pow_id_"+std::to_string(esc_id);
                    _log_pow_map[esc_id]=pow_id;
                    _pow_status_logger->create(pow_id,6);
                }
            }break;
            case iit::ecat::HYQ_KNEE:{
                if(_valve_status_logger==nullptr){
                    _valve_status_logger = XBot::MatLogger2::MakeLogger("/tmp/valve_status_logger", opt);
                    _valve_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                if(_log_valve_map.count(esc_id)==0){
                    std::string valve_sts_id="valve_sts_id_"+std::to_string(esc_id);
                    _log_valve_map[esc_id]=valve_sts_id;
                    _valve_status_logger->create(valve_sts_id,ValvePdoRx::pdo_size);
                }
                
                if(_valves_references_logger==nullptr){
                    _valves_references_logger = XBot::MatLogger2::MakeLogger("/tmp/valves_references_logger");
                    _valves_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_valve_ref_map.count(esc_id)==0){
                    std::string valve_ref_id="valve_ref_id_"+std::to_string(esc_id);
                    _log_valve_ref_map[esc_id]=valve_ref_id;
                    _valves_references_logger->create(valve_ref_id,ValvePdoTx::pdo_size);
                }
                
                
            }break;
            case iit::ecat::HYQ_HPU:{
                if(_pump_status_logger==nullptr){
                    _pump_status_logger = XBot::MatLogger2::MakeLogger("/tmp/pump_status_logger", opt);
                    _pump_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                if(_log_pump_map.count(esc_id)==0){
                    std::string pump_id="pump_id_"+std::to_string(esc_id);
                    _log_pump_map[esc_id]=pump_id;
                    _pump_status_logger->create(pump_id,PumpPdoRx::pdo_size);
                }
                
                if(_pumps_references_logger==nullptr){
                    _pumps_references_logger = XBot::MatLogger2::MakeLogger("/tmp/pumps_references_logger");
                    _pumps_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_pump_ref_map.count(esc_id)==0){
                    std::string pump_ref_id="pump_ref_id_"+std::to_string(esc_id);
                    _log_pump_ref_map[esc_id]=pump_ref_id;
                    _pumps_references_logger->create(pump_ref_id,PumpPdoTx::pdo_size);
                }
                
                
            }break;
                
            default:
                    break;
        }
    }
}

void EcLogger::stop_mat_logger()
{
    _motors_references_logger.reset();
    _valves_references_logger.reset();
    _pumps_references_logger.reset();
    _motors_status_logger.reset();
    _ft_status_logger.reset();
    _pow_status_logger.reset();
    _imu_status_logger.reset();
    _valve_status_logger.reset();
    _pump_status_logger.reset();
}

void EcLogger::log_motors_ref(const std::vector<MR> motors_ref)
{
    if(_motors_references_logger != nullptr){
        for ( const auto &[esc_id,ctrl_type,pos_ref,vel_ref,tor_ref,gain_0,gain_1,gain_2,gain_3,gain_4,op,idx,aux] : motors_ref) {
            if(ctrl_type!=0x00){
                if ((ctrl_type==0x3B)||(ctrl_type==0x71)|| 
                    (ctrl_type==0xD4)||(ctrl_type==0xCC)){
                    if(_log_motor_ref_map.count(esc_id)>0){
                        _motor_ref_eigen(0)=pos_ref;
                        _motor_ref_eigen(1)=vel_ref;
                        _motor_ref_eigen(2)=tor_ref;
                        _motor_ref_eigen(3)=gain_0;
                        _motor_ref_eigen(4)=gain_1;
                        _motor_ref_eigen(5)=gain_2;
                        _motor_ref_eigen(6)=gain_3;
                        _motor_ref_eigen(7)=gain_4;
                        _motor_ref_eigen(8)=op;
                        _motor_ref_eigen(9)=idx;
                        _motor_ref_eigen(10)=aux;
                        _motors_references_logger->add(_log_motor_ref_map[esc_id], _motor_ref_eigen);
                    }
                }
            }
        }
    }
}


void EcLogger::log_motors_sts(const MotorStatusMap motors_sts_map)
{
    if(_motors_status_logger != nullptr){
        for ( const auto &[esc_id, motor_sts] : motors_sts_map) {
            if(_log_motor_sts_map.count(esc_id)>0){
                _motor_sts_eigen(0)=std::get<0>(motor_sts);
                _motor_sts_eigen(1)=std::get<1>(motor_sts);
                _motor_sts_eigen(2)=std::get<2>(motor_sts);
                _motor_sts_eigen(3)=std::get<3>(motor_sts);
                _motor_sts_eigen(4)=std::get<4>(motor_sts);
                _motor_sts_eigen(5)=std::get<5>(motor_sts);
                _motor_sts_eigen(6)=std::get<6>(motor_sts);
                _motor_sts_eigen(7)=std::get<7>(motor_sts);
                _motor_sts_eigen(8)=std::get<8>(motor_sts);
                _motor_sts_eigen(9)=std::get<9>(motor_sts);
                _motor_sts_eigen(10)=std::get<10>(motor_sts);
                _motor_sts_eigen(11)=std::get<11>(motor_sts);
                _motors_status_logger->add(_log_motor_sts_map[esc_id],_motor_sts_eigen);
            }
        }
    }
}

void EcLogger::log_pow_sts(const PwrStatusMap pow_sts_map)
{
    if(_pow_status_logger != nullptr){
        for ( const auto &[esc_id, pow_sts] : pow_sts_map) {
            if(_log_pow_map.count(esc_id)>0){
                _pow_status_logger->add(_log_pow_map[esc_id], pow_sts);
            }
        }
    }
}

void EcLogger::log_ft_sts(const FtStatusMap ft_sts_map)
{
    if(_ft_status_logger != nullptr){
        for ( const auto &[esc_id, ft_sts] : ft_sts_map) {
            if(_log_ft_map.count(esc_id)>0){
                _ft_status_logger->add(_log_ft_map[esc_id],ft_sts);
            }
        }
    }
}

void EcLogger::log_imu_sts(const ImuStatusMap imu_sts_map)
{
    if(_imu_status_logger != nullptr){
        for ( const auto &[esc_id, imu_sts] : imu_sts_map) {
            if(_log_imu_map.count(esc_id)>0){
                _imu_status_logger->add(_log_imu_map[esc_id],imu_sts);
            }
        }
    }
}

void EcLogger::log_valve_ref(const ValveReferenceMap valves_ref)
{
    if(_valves_references_logger != nullptr){
        for ( const auto &[esc_id,valve_tx_pdo] : valves_ref) {
            if(_log_valve_ref_map.count(esc_id)>0){
                if(ValvePdoTx::make_vector_from_tuple(valve_tx_pdo,_valve_tx_v)){
                    _valves_references_logger->add(_log_valve_ref_map[esc_id],_valve_tx_v);
                }
            }
        }
    }
}

void EcLogger::log_valve_sts(const ValveStatusMap valve_sts_map)
{
    if(_valve_status_logger != nullptr){
        for ( const auto &[esc_id, valve_rx_pdo] : valve_sts_map) {
            if(_log_valve_map.count(esc_id)>0){
                if(ValvePdoRx::make_vector_from_tuple(valve_rx_pdo,_valve_rx_v)){
                    _valve_status_logger->add(_log_valve_map[esc_id],_valve_rx_v);
                }
            }
        }
    }
}

void EcLogger::log_pump_ref(const PumpReferenceMap pumps_ref)
{
    if(_pumps_references_logger != nullptr){
        for ( const auto &[esc_id,pump_tx_pdo] : pumps_ref) {
            if(_log_pump_ref_map.count(esc_id)>0){
                if(PumpPdoTx::make_vector_from_tuple(pump_tx_pdo,_pump_tx_v)){
                    _pumps_references_logger->add(_log_pump_ref_map[esc_id],_pump_tx_v);
                }
            }
        }
    }
}

void EcLogger::log_pump_sts(const PumpStatusMap pump_sts_map)
{
    if(_pump_status_logger != nullptr){
        for ( const auto &[esc_id, pump_rx_pdo] : pump_sts_map) {
            if(_log_pump_map.count(esc_id)>0){
                if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,_pump_rx_v)){
                    _pump_status_logger->add(_log_pump_map[esc_id],_pump_rx_v);
                }
            }
        }
    }
}
