#include "ec_rt_trajectory.h"

#include <thread>
#include <chrono>

using namespace std::chrono;

EcRtTrajectory::EcRtTrajectory(int period_ms,
                               EcUtils::EC_CONFIG ec_client_cfg,
                               EcIface::Ptr client):
_ec_client_cfg(ec_client_cfg),                               
_client(client)
{
    // periodic
    struct timespec ts;
    iit::ecat::us2ts(&ts, 1000*period_ms);
    // period.period is a timeval ... tv_usec 
    period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

EcRtTrajectory::~EcRtTrajectory()
{ 
    iit::ecat::print_stat ( s_loop );

    stop();
    
    join();
}

void EcRtTrajectory::th_init ( void * )
{
    // *************** AUTODETECTION *************** //

    _slave_id_vector=_ec_client_cfg.motor_id;
    

    if(_client->retrieve_slaves_info(_slave_info))
    {   
        if(!_slave_info.empty())
        {
            DPRINTF ("AUTODETECTION\n");
            for(int motor_id_index=0;motor_id_index<_slave_id_vector.size();motor_id_index++)
            {
                bool motor_found=false;
                for ( auto &[id, type, pos] : _slave_info ) {
                    if((type==CENT_AC) || (type==LO_PWR_DC_MC))//HP or LP motor
                    {
                        if(id == _ec_client_cfg.motor_id[motor_id_index])
                        {
                            motor_found=true;
                            break;
                        }
                    }
                }
            
                if(!motor_found)
                {
                    throw std::runtime_error("ID: " + std::to_string(_ec_client_cfg.motor_id[motor_id_index]) + "not found");
                }
            }

        }
    }
    
    for(int i=0; i< _slave_id_vector.size();i++)
    {
        int id = _slave_id_vector[i];
        _motors_start.push_back(std::make_tuple(id,_ec_client_cfg.motor_config_map[id].control_mode_type,_ec_client_cfg.motor_config_map[id].gains));
        
        if(_ec_client_cfg.motor_config_map[id].brake_present){
            // queue release brake commands for all motors 
            _brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
        }
    }
        
    // *************** AUTODETECTION *************** //

    bool motor_started=false;

    if(!_motors_start.empty())
    {
        // ************************* START Motors ***********************************//
        DPRINTF ("START ALL MOTORS\n");
        motor_started=_client->start_motors(_motors_start);

        if(motor_started)
        {
            if(!_brake_cmds.empty()){
                // ************************* RELEASE BRAKES ***********************************//
                while(_pdo_aux_cmd_attemps<_max_pdo_aux_cmd_attemps)
                {
                    _pdo_aux_cmd_attemps++;
                    if(!_client->pdo_aux_cmd(_brake_cmds))
                    {
                        DPRINTF ("Cannot perform the release brake command of the motors\n");
                        _pdo_aux_cmd_attemps=_max_pdo_aux_cmd_attemps;
                    }
                    else
                    {
                        std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                        if(_client->pdo_aux_cmd_sts(_brake_cmds))
                        {
                            _send_ref=true;
                            _pdo_aux_cmd_attemps=_max_pdo_aux_cmd_attemps;
                        }
                    }
                }
                // ************************* RELEASE BRAKES ***********************************//
            }
            else{
                _send_ref=true;
            }
        }
        else
        {
            DPRINTF ("Motors not started\n");
        }
            
        // ************************* START Motors ***********************************//
    }
    else
    {
        DPRINTF ("NO MOTORS STARTED\n");
    }
    
#ifdef TEST_EXAMPLES
        if(!_slave_id_vector.empty())
        {
            _send_ref=true;
        }
#endif
    
    //_start_time= steady_clock::now();
    //_time=start_time;

    _hm_time_ms=milliseconds(1000*_ec_client_cfg.homing_time_sec);
    _trj_time_ms=milliseconds(1000*_ec_client_cfg.trajectory_time_sec);
    _time_to_engage_brakes= milliseconds(1000); // Default 1s
    
    _STM_sts="Homing";
    _q_set_trj=_ec_client_cfg.homing_position;
}

void EcRtTrajectory::th_loop ( void * )
{
    if(_send_ref)
    {
        bool client_alive = _client->is_client_alive();
            
        if(!client_alive)
        {
           //stop();
           return;
        }
        
        //auto time_elapsed_ms= duration_cast<milliseconds>(time-start_time);
        
        // Rx "SENSE"
        
        //******************* Power Board Telemetry ********
        auto pow_status_map= _client->get_pow_status();
        float v_batt,v_load,i_load,temp_pcb,temp_heatsink,temp_batt;
        if(!pow_status_map.empty())
        {
            for ( const auto &[esc_id, pow_status] : pow_status_map){
                v_batt =        pow_status[0];
                v_load =        pow_status[1];
                i_load =        pow_status[2];
                temp_pcb =      pow_status[3];
                temp_heatsink=  pow_status[4];
                temp_batt=      pow_status[5];
            }
        }
        //******************* Power Board Telemetry ********
        
        //******************* Motor Telemetry **************
        auto motors_status_map= _client->get_motors_status();
        if(!motors_status_map.empty())
        {
            for ( const auto &[esc_id, motor_status] : motors_status_map){
                try {
                        if(_q_set_trj.count(esc_id))
                        {
                            float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
                            float  motor_temp, board_temp;
                            uint32_t fault,rtt,op_idx_ack;
                            uint32_t cmd_aux_sts,brake_sts,led_sts;
                            std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = motor_status;
                            
                            // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                            brake_sts = cmd_aux_sts & 3; //00 unknown
                                                        //01 release brake 
                                                        //10 enganged brake  
                                                        //11 error
                            led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                            
                            //Closed Loop SENSE for motor velocity
                            _qdot[esc_id] = motor_vel;
                            
                            if(!_first_Rx)
                            {
                                _q_start[esc_id]=motor_pos; // get position at first time
                            }
                        }
                } catch (std::out_of_range oor) {}
            }
            //******************* Motor Telemetry **************
            
            if(_q_start.size() == _q_set_trj.size())
            {
                //Open Loop SENSE
                _first_Rx=true;
            }
            else
            {
                DPRINTF("fatal error: different size of initial position and trajectory vectors");
                
            }
        }
    }
}

    

