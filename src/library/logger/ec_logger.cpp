#include "ec_logger.h"


void EcLogger::start_mat_logger()
{
    // Logger setup
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e4; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    
    _motors_references_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_references_logger");
    _motors_references_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _motors_status_logger = XBot::MatLogger2::MakeLogger("/tmp/motors_status_logger", opt);
    _motors_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _ft6_status_logger = XBot::MatLogger2::MakeLogger("/tmp/ft6_status_logger", opt);
    _ft6_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _pow_status_logger = XBot::MatLogger2::MakeLogger("/tmp/pow_status_logger", opt);
    _pow_status_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
}
void EcLogger::stop_mat_logger()
{
    _motors_references_logger.reset();
    _motors_status_logger.reset();
    _ft6_status_logger.reset();
    _pow_status_logger.reset();
}

void EcLogger::log_motors_ref(std::vector<MR> motors_ref)
{
    if(_motors_references_logger != nullptr)
    {
        if(!motors_ref.empty())
        {
            resize_motors_ref(motors_ref.size());
            int i=0;
            for ( const auto &[bId,ctrl_type,pos_ref,vel_ref,tor_ref,gain_0,gain_1,gain_2,gain_3,gain_4,op,idx,aux] : motors_ref) 
            {
                _pos_ref_eigen(i)=pos_ref;
                _vel_ref_eigen(i)=vel_ref;
                _tor_ref_eigen(i)=tor_ref;
                if(ctrl_type == 0xD4)
                {
                    _GP_eigen(i) = gain_0;
                    _GD_eigen(i) = gain_1;
                }
                else
                {
                    _GP_eigen(i) = gain_0;
                    _GD_eigen(i) = gain_2;
                }
                i++;
            }

            _motors_references_logger->add("pos_ref", _pos_ref_eigen);
            _motors_references_logger->add("vel_ref", _vel_ref_eigen);
            _motors_references_logger->add("tor_ref", _tor_ref_eigen);
            _motors_references_logger->add("GP", _GP_eigen);
            _motors_references_logger->add("GD", _GD_eigen);
            

        }
    }
}

void EcLogger::resize_motors_ref(size_t size)
{
    if(_pos_ref_eigen.size() != size)
    {
        _pos_ref_eigen.resize(size);
        _vel_ref_eigen.resize(size);
        _tor_ref_eigen.resize(size);
        _GP_eigen.resize(size);
        _GD_eigen.resize(size);
    }
}

void EcLogger::log_motors_sts(MotorStatusMap motors_sts_map)
{
    if(_motors_status_logger != nullptr)
    {
        if(!motors_sts_map.empty())
        {
            resize_motors_sts(motors_sts_map.size());
            int i=0;
            for ( const auto &[esc_id, motor_sts] : motors_sts_map) 
            {
                try {
                    float link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,aux;
                    uint32_t fault,rtt,op_idx_ack,cmd_aux_sts;
                    std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = motor_sts;
                    
                    _link_pos_eigen(i)=link_pos;
                    _motor_pos_eigen(i)=motor_pos;
                    _link_vel_eigen(i)=link_vel;
                    _motor_vel_eigen(i)=motor_vel;
                    _tor_eigen(i)=torque;
                    _motor_temp_eigen(i)=motor_temp;
                    _board_temp_eigen(i)=board_temp;
                    _fault_eigen(i)=fault;
                    
                    i++;

                } catch (std::out_of_range oor) {}
            }


            _motors_status_logger->add("link_pos", _link_pos_eigen);
            _motors_status_logger->add("motor_pos", _motor_pos_eigen);
            _motors_status_logger->add("link_vel", _link_vel_eigen);
            _motors_status_logger->add("motor_vel", _motor_vel_eigen);
            _motors_status_logger->add("torque", _tor_eigen);
            _motors_status_logger->add("motor_temp", _motor_temp_eigen);
            _motors_status_logger->add("board_temp", _board_temp_eigen);
            _motors_status_logger->add("fault", _fault_eigen);
        }
    }
}

void EcLogger::resize_motors_sts(size_t size)
{
    if(_link_pos_eigen.size() != size)
    {
        _link_pos_eigen.resize(size);
        _motor_pos_eigen.resize(size);
        _link_vel_eigen.resize(size);
        _motor_vel_eigen.resize(size);
        _tor_eigen.resize(size);
        _motor_temp_eigen.resize(size);
        _board_temp_eigen.resize(size);
        _fault_eigen.resize(size);
    }
}

void EcLogger::log_pow_sts(PwrStatusMap pow_sts_map)
{
    if(_pow_status_logger != nullptr)
    {
        if(!pow_sts_map.empty())
        {
            resize_pow_sts(pow_sts_map.size());
            int i=0;
            for ( const auto &[esc_id, pow_sts] : pow_sts_map) 
            {
                _v_batt_eigen(i)=pow_sts[0];
                _v_load_eigen(i)=pow_sts[1];
                _i_load_eigen(i)=pow_sts[2];
                _temp_pcb_eigen(i)=pow_sts[3];
                _temp_heatsink_eigen(i)=pow_sts[4];
                _temp_batt_eigen(i)=pow_sts[5];
                i++;
            }

            _pow_status_logger->add("v_batt", _v_batt_eigen);
            _pow_status_logger->add("v_load", _v_load_eigen);
            _pow_status_logger->add("i_load", _i_load_eigen);
            _pow_status_logger->add("temp_pcb", _temp_pcb_eigen);
            _pow_status_logger->add("temp_heatsink", _temp_heatsink_eigen);
            _pow_status_logger->add("temp_batt", _temp_batt_eigen);
        }
    }
}

void EcLogger::resize_pow_sts(size_t size)
{
    if(_v_batt_eigen.size() != size)
    {
        _v_batt_eigen.resize(size);
        _v_load_eigen.resize(size);
        _i_load_eigen.resize(size);

        _temp_pcb_eigen.resize(size);
        _temp_heatsink_eigen.resize(size);
        _temp_batt_eigen.resize(size);
    }
}


void EcLogger::log_ft6_sts(FtStatusMap ft6_sts_map)
{
    if(_ft6_status_logger != nullptr)
    {
        if(!ft6_sts_map.empty())
        {
            resize_ft6_sts(ft6_sts_map.size());
            int i=0;
            for ( const auto &[esc_id, ft6_sts] : ft6_sts_map) 
            {
                _force_x_eigen(i)=ft6_sts[0];
                _force_y_eigen(i)=ft6_sts[1];
                _force_z_eigen(i)=ft6_sts[2];
                _torque_x_eigen(i)=ft6_sts[3];
                _torque_y_eigen(i)=ft6_sts[4];
                _torque_z_eigen(i)=ft6_sts[5];
                i++;
            }

            _ft6_status_logger->add("force_x", _force_x_eigen);
            _ft6_status_logger->add("force_y", _force_y_eigen);
            _ft6_status_logger->add("force_z", _force_z_eigen);
            _ft6_status_logger->add("torque_x", _torque_x_eigen);
            _ft6_status_logger->add("torque_y", _torque_y_eigen);
            _ft6_status_logger->add("torque_z", _torque_z_eigen);
        }
    }
}

void EcLogger::resize_ft6_sts(size_t size)
{
    if(_force_x_eigen.size() != size)
    {
        _force_x_eigen.resize(size);
        _force_y_eigen.resize(size);
        _force_z_eigen.resize(size);

        _torque_x_eigen.resize(size);
        _torque_y_eigen.resize(size);
        _torque_z_eigen.resize(size);
    }
}
