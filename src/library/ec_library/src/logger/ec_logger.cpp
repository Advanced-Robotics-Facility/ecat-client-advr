#include "logger/ec_logger.h"

EcLogger::EcLogger()
{
    _motor_rx_v.resize(MotorPdoRx::pdo_size);
    _motor_tx_v.resize(MotorPdoTx::pdo_size);
    _pow_rx_v.resize(PowPdoRx::pdo_size);
    _ft_rx_v.resize(FtPdoRx::pdo_size);
    _imu_rx_v.resize(ImuPdoRx::pdo_size);
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
    opt.default_buffer_size = 86400000; // set default buffer size of 24h
    opt.enable_compression = true; // enable ZLIB compression
    
    for ( auto &[esc_id, esc_type, pos] : _slave_descr ) {
        switch ( esc_type  )
        {
            case iit::ecat::CENT_AC :
            case iit::ecat::LO_PWR_DC_MC :
            case iit::ecat::SYNAPTICON_v5_0:
            case iit::ecat::SYNAPTICON_v5_1:
            case iit::ecat::AMC_FLEXPRO:{
                if(_motors_status_logger==nullptr){
                    _motors_status_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_status_logger", opt);
                    _motors_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_motor_sts_map.count(esc_id)==0){
                    std::string motor_sts_id="motor_sts_id_"+std::to_string(esc_id);
                    _log_motor_sts_map[esc_id]=motor_sts_id;
                    _motors_status_logger->create(motor_sts_id,MotorPdoRx::pdo_size);
                }
                
                if(_motors_references_logger==nullptr){
                    _motors_references_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_references_logger");
                    _motors_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
                }
                
                if(_log_motor_ref_map.count(esc_id)==0){
                    std::string motor_ref_id="motor_ref_id_"+std::to_string(esc_id);
                    _log_motor_ref_map[esc_id]=motor_ref_id;
                    _motors_references_logger->create(motor_ref_id,MotorPdoTx::pdo_size);
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
                    _ft_status_logger->create(ft_id,FtPdoRx::pdo_size);
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
                    _imu_status_logger->create(imu_id,ImuPdoRx::pdo_size);
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
                    _pow_status_logger->create(pow_id,PowPdoRx::pdo_size);
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

void EcLogger::log_motors_ref(const MotorReferenceMap& motors_ref)
{
    if(_motors_references_logger != nullptr){
        for ( const auto &[esc_id,motor_tx_pdo] : motors_ref) {
            if(_log_motor_ref_map.count(esc_id)>0){
                if(MotorPdoTx::make_vector_from_tuple(motor_tx_pdo,_motor_tx_v)){
                    _motors_references_logger->add(_log_motor_ref_map[esc_id], _motor_tx_v);
                }
            }
        }
    }
}


void EcLogger::log_motors_sts(const MotorStatusMap& motors_sts_map)
{
    if(_motors_status_logger != nullptr){
        for ( const auto &[esc_id, motor_rx_pdo] : motors_sts_map) {
            if(_log_motor_sts_map.count(esc_id)>0){
                if(MotorPdoRx::make_vector_from_tuple(motor_rx_pdo,_motor_rx_v)){
                    _motors_status_logger->add(_log_motor_sts_map[esc_id],_motor_rx_v);
                }
            }
        }
    }
}

void EcLogger::log_pow_sts(const PwrStatusMap& pow_sts_map)
{
    if(_pow_status_logger != nullptr){
        for ( const auto &[esc_id, pow_rx_pdo] : pow_sts_map) {
            if(_log_pow_map.count(esc_id)>0){
                if(PowPdoRx::make_vector_from_tuple(pow_rx_pdo,_pow_rx_v)){
                    _pow_status_logger->add(_log_pow_map[esc_id], _pow_rx_v);
                }
            }
        }
    }
}

void EcLogger::log_ft_sts(const FtStatusMap& ft_sts_map)
{
    if(_ft_status_logger != nullptr){
        for ( const auto &[esc_id, ft_rx_pdo] : ft_sts_map) {
            if(_log_ft_map.count(esc_id)>0){
                if(FtPdoRx::make_vector_from_tuple(ft_rx_pdo,_ft_rx_v)){
                    _ft_status_logger->add(_log_ft_map[esc_id],_ft_rx_v);
                }
            }
        }
    }
}

void EcLogger::log_imu_sts(const ImuStatusMap& imu_sts_map)
{
    if(_imu_status_logger != nullptr){
        for ( const auto &[esc_id, imu_rx_pdo] : imu_sts_map) {
            if(_log_imu_map.count(esc_id)>0){
                if(ImuPdoRx::make_vector_from_tuple(imu_rx_pdo,_imu_rx_v)){
                    _imu_status_logger->add(_log_imu_map[esc_id],_imu_rx_v);
                }
            }
        }
    }
}

void EcLogger::log_valve_ref(const ValveReferenceMap& valves_ref)
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

void EcLogger::log_valve_sts(const ValveStatusMap& valve_sts_map)
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

void EcLogger::log_pump_ref(const PumpReferenceMap& pumps_ref)
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

void EcLogger::log_pump_sts(const PumpStatusMap& pump_sts_map)
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
