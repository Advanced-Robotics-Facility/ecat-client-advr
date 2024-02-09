#include "logger/ec_logger.h"

EcLogger::EcLogger()
{
    _motor_ref_eigen.resize(11);
    _motor_sts_eigen.resize(12);
}

void EcLogger::start_mat_logger()
{
    // Logger setup
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e4; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    
    _motors_references_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_references_logger");
    _motors_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    
    
    _set_motors_references_logger = XBot::MatLogger2::MakeLogger("/tmp/set_motors_references_logger");
    _set_motors_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    

    _motors_status_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_status_logger", opt);
    _motors_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _ft_status_logger = XBot::MatLogger2::MakeLogger("/tmp/ft6_status_logger", opt);
    _ft_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _pow_status_logger = XBot::MatLogger2::MakeLogger("/tmp/pow_status_logger", opt);
    _pow_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _imu_status_logger = XBot::MatLogger2::MakeLogger("/tmp/imu_status_logger", opt);
    _imu_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
}
void EcLogger::stop_mat_logger()
{
    _motors_references_logger.reset();
    _motors_status_logger.reset();
    _ft_status_logger.reset();
    _pow_status_logger.reset();
    _imu_status_logger.reset();
}

void EcLogger::add_motors_ref(std::vector<MR> motors_ref,XBot::MatLogger2::Ptr logger)
{
    if(logger != nullptr){
        for ( const auto &[esc_id,ctrl_type,pos_ref,vel_ref,tor_ref,gain_0,gain_1,gain_2,gain_3,gain_4,op,idx,aux] : motors_ref) {
            if(ctrl_type!=0x00){
                if ((ctrl_type==0x3B)||(ctrl_type==0x71)|| 
                    (ctrl_type==0xD4)||(ctrl_type==0xCC)){
                    std::string motor_ref_id="motor_ref_id_"+std::to_string(esc_id);
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
                    logger->add(motor_ref_id, _motor_ref_eigen);
                }
            }
        }
    }
}


void EcLogger::log_motors_ref(std::vector<MR> motors_ref)
{
    add_motors_ref(motors_ref,_motors_references_logger);
}

void EcLogger::log_set_motors_ref(std::vector<MR> motors_ref)
{
    add_motors_ref(motors_ref,_set_motors_references_logger);
}

void EcLogger::log_motors_sts(MotorStatusMap motors_sts_map)
{
    if(_motors_status_logger != nullptr){
        for ( const auto &[esc_id, motor_sts] : motors_sts_map) {
            std::string motor_sts_id="motor_sts_id_"+std::to_string(esc_id);
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
            
            _motors_status_logger->add(motor_sts_id,_motor_sts_eigen);
        }
    }
}

void EcLogger::log_pow_sts(PwrStatusMap pow_sts_map)
{
    if(_pow_status_logger != nullptr){
        for ( const auto &[esc_id, pow_sts] : pow_sts_map) {
            std::string pow_id="pow_id_"+std::to_string(esc_id);
            _pow_status_logger->add(pow_id, pow_sts);
        }
    }
}

void EcLogger::log_ft_sts(const FtStatusMap ft_sts_map)
{
    if(_ft_status_logger != nullptr){
        for ( const auto &[esc_id, ft_sts] : ft_sts_map) {
            std::string ft_id="ft_id_"+std::to_string(esc_id);
            _ft_status_logger->add(ft_id,ft_sts);
        }
    }
}

void EcLogger::log_imu_sts(const ImuStatusMap imu_sts_map)
{
    if(_imu_status_logger != nullptr){
        for ( const auto &[esc_id, imu_sts] : imu_sts_map) {
            std::string imu_id="imu_id_"+std::to_string(esc_id);
            _imu_status_logger->add(imu_id,imu_sts);
        }
    }
}
