#include "logger/ec_logger.h"

EcLogger::EcLogger()
{
    // Logger setup
    _log_opt.default_buffer_size = 1e4; // set default buffer size of 24h
    _log_opt.enable_compression = true; // enable ZLIB compression
    _log_dir="/tmp/";
}

void EcLogger::init_mat_logger(SSI slave_descr)
{
    _slave_descr.clear();
    _slave_descr=slave_descr;
}

void EcLogger::create_logger(std::string log_name,
                             int esc_id,
                             std::string log_esc_type,
                             int log_row)
{
    if(!_log_map[log_name]){
        std::string log_file=_log_dir+log_name;
        _log_map[log_name] = XBot::MatLogger2::MakeLogger(log_file,_log_opt);
        _log_appender->add_logger(_log_map[log_name]);
    }

    auto log_esc_map = &_log_stsEsc_map;
    auto log_row_map = &_log_stsRow_map;
    if (log_name.find("reference") != std::string::npos) {
        log_esc_map = &_log_refEsc_map;
        log_row_map = &_log_refRow_map;
    }

    if(log_esc_map->count(esc_id)==0){       
        std::string log_esc=log_esc_type+"_"+std::to_string(esc_id);
        log_esc_map->insert(std::pair{esc_id,log_esc});
        log_row_map->insert(std::pair{esc_id,std::vector<float>(log_row)});
        _log_map[log_name]->create(log_esc,log_row);
    }

}

void EcLogger::start_mat_logger()
{

    stop_mat_logger();

    if(!_slave_descr.empty()){
        _log_appender = XBot::MatAppender::MakeInstance();
    }
    
    for ( auto &[esc_id, esc_type, pos] : _slave_descr ) {
        switch ( esc_type  )
        {
            case iit::ecat::CENT_AC :
            case iit::ecat::LO_PWR_DC_MC :
            case iit::ecat::SYNAPTICON_v5_0:
            case iit::ecat::SYNAPTICON_v5_1:{
                create_logger("motor_status_logger",esc_id,"motor_sts_id",MotorPdoRx::pdo_size);
                create_logger("motor_reference_logger",esc_id,"motor_ref_id",MotorPdoTx::pdo_size);
            }break;
            case iit::ecat::FT6_MSP432:{
                create_logger("ft_status_logger",esc_id,"ft_id",FtPdoRx::pdo_size);
            }break; 
            case iit::ecat::IMU_ANY :{
                create_logger("imu_status_logger",esc_id,"imu_id",ImuPdoRx::pdo_size);
            }break;
            case iit::ecat::POW_F28M36_BOARD :{
                create_logger("pow_status_logger",esc_id,"pow_id",PowPdoRx::pdo_size);
            }break;
            case iit::ecat::HYQ_KNEE:{
                create_logger("valve_status_logger",esc_id,"valve_sts_id",ValvePdoRx::pdo_size);
                create_logger("valve_reference_logger",esc_id,"valve_ref_id",ValvePdoTx::pdo_size);
            }break;
            case iit::ecat::HYQ_HPU:{
                create_logger("pump_status_logger",esc_id,"pump_sts_id",PumpPdoRx::pdo_size);
                create_logger("pump_reference_logger",esc_id,"pump_ref_id",PumpPdoTx::pdo_size);
            }break;
                
            default:
                    break;
        }
    }

    if(_log_appender){
        _log_appender->start_flush_thread();
    }
}

void EcLogger::stop_mat_logger()
{
    for(auto &[log_name,log]:_log_map){
        log.reset();
    }
    
    _log_map.clear();
    _log_stsEsc_map.clear();
    _log_refEsc_map.clear();
    _log_stsRow_map.clear();
    _log_refRow_map.clear();
    _log_appender.reset();
}

void EcLogger::log_motors_ref(const MotorReferenceMap& motors_ref)
{
    if(_log_map["motor_reference_logger"]){
        for ( const auto &[esc_id,motor_tx_pdo] : motors_ref) {
            if(_log_refEsc_map.count(esc_id)>0){
                if(MotorPdoTx::make_vector_from_tuple(motor_tx_pdo,_log_refRow_map[esc_id])){
                    _log_map["motor_reference_logger"]->add(_log_refEsc_map[esc_id], _log_refRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_motors_sts(const MotorStatusMap& motors_sts_map)
{
    if(_log_map["motor_status_logger"]){
        for ( const auto &[esc_id, motor_rx_pdo] : motors_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(MotorPdoRx::make_vector_from_tuple(motor_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["motor_status_logger"]->add(_log_stsEsc_map[esc_id],_log_stsRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_pow_sts(const PwrStatusMap& pow_sts_map)
{
    if(_log_map["pow_status_logger"]){
        for ( const auto &[esc_id, pow_rx_pdo] : pow_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(PowPdoRx::make_vector_from_tuple(pow_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["pow_status_logger"]->add(_log_stsEsc_map[esc_id], _log_stsRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_ft_sts(const FtStatusMap& ft_sts_map)
{
    if(_log_map["ft_status_logger"]){
        for ( const auto &[esc_id, ft_rx_pdo] : ft_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(FtPdoRx::make_vector_from_tuple(ft_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["ft_status_logger"]->add(_log_stsEsc_map[esc_id],_log_stsRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_imu_sts(const ImuStatusMap& imu_sts_map)
{
    if(_log_map["imu_status_logger"]){
        for ( const auto &[esc_id, imu_rx_pdo] : imu_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(ImuPdoRx::make_vector_from_tuple(imu_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["imu_status_logger"]->add(_log_stsEsc_map[esc_id],_log_stsRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_valve_ref(const ValveReferenceMap& valves_ref)
{
    if(_log_map["valve_reference_logger"]){
        for ( const auto &[esc_id,valve_tx_pdo] : valves_ref) {
            if(_log_refEsc_map.count(esc_id)>0){
                if(ValvePdoTx::make_vector_from_tuple(valve_tx_pdo,_log_refRow_map[esc_id])){
                    _log_map["valve_reference_logger"]->add(_log_refEsc_map[esc_id],_log_refRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_valve_sts(const ValveStatusMap& valve_sts_map)
{
    if(_log_map["valve_status_logger"]){
        for ( const auto &[esc_id, valve_rx_pdo] : valve_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(ValvePdoRx::make_vector_from_tuple(valve_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["valve_status_logger"]->add(_log_stsEsc_map[esc_id],_log_stsRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_pump_ref(const PumpReferenceMap& pumps_ref)
{
    if(_log_map["pump_reference_logger"]){
        for ( const auto &[esc_id,pump_tx_pdo] : pumps_ref) {
            if(_log_refEsc_map.count(esc_id)>0){
                if(PumpPdoTx::make_vector_from_tuple(pump_tx_pdo,_log_refRow_map[esc_id])){
                    _log_map["pump_reference_logger"]->add(_log_refEsc_map[esc_id],_log_refRow_map[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_pump_sts(const PumpStatusMap& pump_sts_map)
{
    if(_log_map["pump_status_logger"]){
        for ( const auto &[esc_id, pump_rx_pdo] : pump_sts_map) {
            if(_log_stsEsc_map.count(esc_id)>0){
                if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,_log_stsRow_map[esc_id])){
                    _log_map["pump_status_logger"]->add(_log_stsEsc_map[esc_id],_log_stsRow_map[esc_id]);
                }
            }
        }
    }
}
