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
}

void EcRtTrajectory::th_loop ( void * )
{
    if(_send_ref)
    {
        
    }
}

    

