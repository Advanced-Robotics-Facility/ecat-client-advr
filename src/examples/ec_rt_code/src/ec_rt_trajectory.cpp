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

    std::vector<int> slave_id_vector=_ec_client_cfg.motor_id;
    
    MST motors_start = {};
    PAC brake_cmds = {};
    PAC led_cmds = {};
    
    SSI slave_info;

    if(_client->retrieve_slaves_info(slave_info))
    {   
        if(!slave_info.empty())
        {
            std::cout << "AUTODETECTION" << std::endl;
            for(int motor_id_index=0;motor_id_index<slave_id_vector.size();motor_id_index++)
            {
                bool motor_found=false;
                for ( auto &[id, type, pos] : slave_info ) {
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
    
    for(int i=0; i< slave_id_vector.size();i++)
    {
        int id = slave_id_vector[i];
        motors_start.push_back(std::make_tuple(id,_ec_client_cfg.control_mode_type,_ec_client_cfg.gains));
        
        // queue release brake commands for all motors 
        brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
        
        for(int k=0 ; k < _ec_client_cfg.slave_id_led.size();k++)  // led on only for the last slaves on the chain
        {
            if(id == _ec_client_cfg.slave_id_led[k])
            {
                led_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::LED_ON)));
            }
        }
    }
        
    // *************** AUTODETECTION *************** //

    bool send_ref=false;
    bool stop_motors=false;
    bool motor_started=false;
    const int max_pdo_aux_cmd_attemps=3; // 3 times to release/engage or LED ON/OFF command
    int pdo_aux_cmd_attemps=0; // 3 times to release/engage or LED ON/OFF command

    if(!motors_start.empty())
    {
        // ************************* START Motors ***********************************//
        std::cout << "START ALL MOTORS" << std::endl;
        motor_started=_client->start_motors(motors_start);

        if(motor_started)
        {
            // ************************* RELEASE BRAKES ***********************************//
            while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
            {
                pdo_aux_cmd_attemps++;
                if(!_client->pdo_aux_cmd(brake_cmds))
                {
                    std::cout << "Cannot perform the release brake command of the motors" << std::endl;
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                }
                else
                {
                    std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                    if(_client->pdo_aux_cmd_sts(brake_cmds))
                    {
                        send_ref=true;
                        pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    }
                }
            }
            // ************************* RELEASE BRAKES ***********************************//
        }
        else
        {
            std::cout << "Motors not started" << std::endl;
        }
            
        // ************************* START Motors ***********************************//

        // ************************* SWITCH ON LEDs ***********************************//
        pdo_aux_cmd_attemps=0;
        while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
        {
            pdo_aux_cmd_attemps++;
            if(!_client->pdo_aux_cmd(led_cmds))
            {
                std::cout << "Cannot perform the led on command of the motors"<< std::endl;
                pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
            }
            else
            {
                std::this_thread::sleep_for(100ms);
                if(_client->pdo_aux_cmd_sts(led_cmds))
                {
                    std::cout << "Switched ON the LEDs " << std::endl;
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                }
            }
        }
        // ************************* SWITCH ON LEDs ***********************************//
    }
    else
    {
        std::cout << "NO MOTORS STARTED" << std::endl;
    }
}

void EcRtTrajectory::th_loop ( void * )
{

}

    

