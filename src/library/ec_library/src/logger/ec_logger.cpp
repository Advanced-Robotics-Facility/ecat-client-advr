#include "logger/ec_logger.h"

EcLogger::EcLogger(bool compression_enabled)
{
    // Logger setup
    _logger_dir="/tmp/";
    _logger_opt.default_buffer_size = 1e4;
    if(compression_enabled){
        _logger_opt.default_buffer_size  = 86400000; // set default buffer size of 24h
    }
    _logger_opt.enable_compression = compression_enabled; // enable ZLIB compression
}

void EcLogger::init_mat_logger(SSI slave_descr)
{
    _slave_descr.clear();
    _slave_descr=slave_descr;
}

void EcLogger::create_logger(std::string logger_name,
                             int esc_id,
                             std::string logger_entry_type,
                             int logger_row)
{
    if(_logger_map.count(logger_name)==0){
        _logger_map[logger_name] = std::make_shared<EcLogger::LOGGER_INFO>();
        std::string logger_file=_logger_dir+logger_name;
        _logger_map[logger_name]->logger = XBot::MatLogger2::MakeLogger(logger_file,_logger_opt);
        _appender->add_logger(_logger_map[logger_name]->logger);
    }

    if(_logger_map[logger_name]->logger_entry.count(esc_id)==0){  
        std::string logger_entry=logger_entry_type+"_"+std::to_string(esc_id);
        _logger_map[logger_name]->logger_entry[esc_id]=logger_entry;
        _logger_map[logger_name]->logger_row[esc_id].resize(logger_row);
    }
}

void EcLogger::start_mat_logger()
{

    stop_mat_logger();

    if(!_slave_descr.empty()){
        _appender = XBot::MatAppender::MakeInstance();
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

    if(_appender){
        _appender->start_flush_thread();
    }
}

void EcLogger::stop_mat_logger()
{
    for(auto &[logger_name,logger_info]:_logger_map){
        logger_info->logger.reset();
        logger_info->logger_entry.clear();
        logger_info->logger_row.clear();
        logger_info.reset();
    }
    
    _logger_map.clear();
    _appender.reset();
}

void EcLogger::log_motor_status(const MotorStatusMap& motor_status_map)
{
    if(_logger_map.count("motor_status_logger")>0){
        auto logger_info = _logger_map["motor_status_logger"];
        for ( const auto &[esc_id,motor_rx_pdo] : motor_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(MotorPdoRx::make_vector_from_tuple(motor_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_motor_reference(const MotorReferenceMap& motor_reference_map)
{
    if(_logger_map.count("motor_reference_logger")>0){
        auto logger_info = _logger_map["motor_reference_logger"];
        for ( const auto &[esc_id,motor_tx_pdo] : motor_reference_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(MotorPdoTx::make_vector_from_tuple(motor_tx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}


void EcLogger::log_pow_status(const PwrStatusMap& pow_status_map)
{
    if(_logger_map.count("pow_status_logger")>0){
        auto logger_info = _logger_map["pow_status_logger"];
        for ( const auto &[esc_id,pow_rx_pdo] : pow_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(PowPdoRx::make_vector_from_tuple(pow_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_ft_status(const FtStatusMap& ft_status_map)
{
    if(_logger_map.count("ft_status_logger")>0){
        auto logger_info = _logger_map["ft_status_logger"];
        for ( const auto &[esc_id,ft_rx_pdo] : ft_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(FtPdoRx::make_vector_from_tuple(ft_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_imu_status(const ImuStatusMap& imu_status_map)
{
    if(_logger_map.count("imu_status_logger")>0){
        auto logger_info = _logger_map["imu_status_logger"];
        for ( const auto &[esc_id,imu_rx_pdo] : imu_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(ImuPdoRx::make_vector_from_tuple(imu_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_valve_status(const ValveStatusMap& valve_status_map)
{
    if(_logger_map.count("valve_status_logger")>0){
        auto logger_info = _logger_map["valve_status_logger"];
        for ( const auto &[esc_id,valve_rx_pdo] : valve_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(ValvePdoRx::make_vector_from_tuple(valve_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_valve_reference(const ValveReferenceMap& valve_reference_map)
{
    if(_logger_map.count("valve_reference_logger")>0){
        auto logger_info = _logger_map["valve_reference_logger"];
        for ( const auto &[esc_id,valve_tx_pdo] : valve_reference_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(ValvePdoTx::make_vector_from_tuple(valve_tx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_pump_status(const PumpStatusMap& pump_status_map)
{
    if(_logger_map.count("pump_status_logger")>0){
        auto logger_info = _logger_map["pump_status_logger"];
        for ( const auto &[esc_id,pump_rx_pdo] : pump_status_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}

void EcLogger::log_pump_reference(const PumpReferenceMap& pump_reference_map)
{
    if(_logger_map.count("pump_reference_logger")>0){
        auto logger_info = _logger_map["pump_reference_logger"];
        for ( const auto &[esc_id,pump_tx_pdo] : pump_reference_map) {
            if(logger_info->logger_entry.count(esc_id)>0){
                if(PumpPdoTx::make_vector_from_tuple(pump_tx_pdo,logger_info->logger_row[esc_id])){
                    logger_info->logger->add(logger_info->logger_entry[esc_id],logger_info->logger_row[esc_id]);
                }
            }
        }
    }
}