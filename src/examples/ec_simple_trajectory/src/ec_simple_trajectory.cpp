#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "ec_utils.h"

using namespace std::chrono;

int main()
{
    EcUtils::Ptr ec_client_utils;
    EcUtils::EC_CONFIG ec_client_cfg;

    // Find the ec client configuration environment variable for setting.
    char* ec_client_cfg_path;
    ec_client_cfg_path = getenv ("EC_CLIENT_CFG");

    if (ec_client_cfg_path==NULL)
    {
        std::cout << "EC Client configuration is not found"
                     ", please setup it using the environment variable with name: EC_CLIENT_CFG " << std::endl;
    }
    else
    {
        try{
            auto ec_client_cfg_file = YAML::LoadFile(ec_client_cfg_path);
            ec_client_utils=std::make_shared<EcUtils>(ec_client_cfg_file);
            ec_client_cfg = ec_client_utils->get_ec_cfg();
        }catch(std::exception &ex){
            std::cout << "Error on ec client config file" << std::endl;
            std::cout << ex.what() << std::endl;
        return 1;
        }
        
        std::vector<int> slave_id_vector;
        
// ********************* TEST ****************************////
#ifdef TEST_EXAMPLES
        for (auto &[id,value]: ec_client_cfg.homing_position)
        {
            slave_id_vector.push_back(id);
        }
#endif
// ********************* TEST ****************************///

        // *************** START CLIENT  *************** //
        EcIface::Ptr client=ec_client_utils->make_ec_iface();
        
        auto period_ms_time=milliseconds(ec_client_cfg.period_ms);
        
        // *************** START CLIENT  *************** //
                    
        
        // *************** AUTODETECTION *************** //
        
        MST motors_start = {};
        PAC brake_cmds = {};
        PAC led_cmds = {};
        
        SSI slave_info;

        if(client->retrieve_slaves_info(slave_info))
        {   
            if(!slave_info.empty())
            {
                std::cout << "AUTODETECTION" << std::endl;
                slave_id_vector.clear();  // clear in case of TEST flag is ON
                for ( auto &[id, type, pos] : slave_info ) {
                    if((type==CENT_AC) || (type==LO_PWR_DC_MC))//HP or LP motor
                    {
                        if(ec_client_cfg.homing_position.count(id))
                        {
                            slave_id_vector.push_back(id);
                        }
                    }
                }
            }
        }
        
        for(int i=0; i< slave_id_vector.size();i++)
        {
            int id = slave_id_vector[i];
            motors_start.push_back(std::make_tuple(id,ec_client_cfg.control_mode_type,ec_client_cfg.gains));
            
            // queue release brake commands for all motors 
            brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
            
            for(int k=0 ; k < ec_client_cfg.slave_id_led.size();k++)  // led on only for the last slaves on the chain
            {
                if(id == ec_client_cfg.slave_id_led[k])
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
            motor_started=client->start_motors(motors_start);

            if(motor_started)
            {
                // ************************* RELEASE BRAKES ***********************************//
                while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                {
                    pdo_aux_cmd_attemps++;
                    if(!client->pdo_aux_cmd(brake_cmds))
                    {
                        std::cout << "Cannot perform the release brake command of the motors" << std::endl;
                        pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    }
                    else
                    {
                        std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                        if(client->pdo_aux_cmd_sts(brake_cmds))
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
                if(!client->pdo_aux_cmd(led_cmds))
                {
                    std::cout << "Cannot perform the led on command of the motors"<< std::endl;
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                }
                else
                {
                    std::this_thread::sleep_for(100ms);
                    if(client->pdo_aux_cmd_sts(led_cmds))
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

#ifdef TEST_EXAMPLES
        if(!slave_id_vector.empty())
        {
            send_ref=true;
        }
#endif
                
        if(send_ref)
        {
            auto start_time= steady_clock::now();
            auto time=start_time;
            
            auto hm_time_ms=milliseconds(1000*ec_client_cfg.homing_time_sec);
            auto trj_time_ms=milliseconds(1000*ec_client_cfg.trajectory_time_sec);
            
            auto time_to_engage_brakes = milliseconds(1000); // Default 1s
            
            bool run=true;
            bool first_Rx=false;
            
            std::string STM_sts="Homing";
            std::map<double,double> q_set_trj=ec_client_cfg.homing_position;
            std::map<double,double> q_ref,q_start,qdot;

            auto set_trj_time_ms=hm_time_ms;
            int trajectory_counter=0;
            bool motors_vel_check=false;
            bool led_off_req=false;
            
            std::vector<MR> motors_ref;
        
            while (run)
            {
                bool client_alive = client->is_client_alive();
                
                if(!client_alive)
                {
                    break;
                }
                
                auto time_elapsed_ms= duration_cast<milliseconds>(time-start_time);
                
                // Rx "SENSE"
                
                //******************* Power Board Telemetry ********
                auto pow_status_map= client->get_pow_status();
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
                auto motors_status_map= client->get_motors_status();
                if(!motors_status_map.empty())
                {
                    for ( const auto &[esc_id, motor_status] : motors_status_map){
                        try {
                                if(q_set_trj.count(esc_id))
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
                                    
                                    //std::cout << "ID: " << esc_id << "  Brake status : " << brake_sts << std::endl;
                                    //std::cout << "ID: " << esc_id << "  LED status : " << led_sts << std::endl;
                                    
                                    //Closed Loop SENSE for motor velocity
                                    qdot[esc_id] = motor_vel;
                                    
                                    if(!first_Rx)
                                    {
                                        q_start[esc_id]=motor_pos; // get position at first time
                                    }
                                }
                        } catch (std::out_of_range oor) {}
                    }
                    //******************* Motor Telemetry **************
                    
                    if(q_start.size() == q_set_trj.size())
                    {
                        //Open Loop SENSE
                        first_Rx=true;
                    }
                    else
                    {
                        throw std::runtime_error("fatal error: different size of initial position and trajectory vectors");
                    }
                }
                    
#ifdef TEST_EXAMPLES
                if(!first_Rx)
                {
                    first_Rx=true;
                    for(int i=0; i<q_set_trj.size();i++)
                    {
                        int id=slave_id_vector[i];
                        q_start[id]=0.0;
                        qdot[id] = 0.0;
                    }
                }
#endif
                
                if((STM_sts=="Homing")||
                   (STM_sts=="Trajectory"))
                {
                    motors_ref.clear();  //clear old trajectory
                    // define a simplistic linear trajectory
                    auto tau= std::chrono::duration<double> (time_elapsed_ms) / std::chrono::duration<double> (set_trj_time_ms);

                    // quintic poly 6t^5 - 15t^4 + 10t^3
                    auto alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
                    //std::cout << "Alpha: "<< alpha << std::endl;
                    // interpolate
                    for(int i=0; i<q_set_trj.size();i++)
                    {
                        int id=slave_id_vector[i];
                        //std::cout << id << std::endl;
                        if(q_set_trj.count(id)>0)
                        {
                            q_ref[id] = q_start[id] + alpha * (q_set_trj[id] - q_start[id]);
                            //std::cout << q_ref[id] << " " ;
                        }
                    }
                    //std::cout << " " << std::endl;
                    for(int i=0; i<slave_id_vector.size();i++)
                    {
                        auto id=slave_id_vector[i];
                        if(q_ref.count(id)>0)
                        {
                            auto pos= q_ref[id];
                            
                            motors_ref.push_back(std::make_tuple(id, //bId
                                                                    ec_client_cfg.control_mode_type, //ctrl_type
                                                                    pos, //pos_ref
                                                                    0.0, //vel_ref
                                                                    0.0, //tor_ref
                                                                    ec_client_cfg.gains[0], //gain_1
                                                                    ec_client_cfg.gains[1], //gain_2
                                                                    ec_client_cfg.gains[2], //gain_3
                                                                    ec_client_cfg.gains[3], //gain_4
                                                                    ec_client_cfg.gains[4], //gain_5
                                                                    1, // op means NO_OP
                                                                    0, // idx
                                                                    0  // aux
                                                                ));
                        }
                    }
                }
                
                if(motors_ref.size() != q_set_trj.size())
                {
                    throw std::runtime_error("fatal error: different size of reference and trajectory vectors");
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
                
                if(!motors_ref.empty())
                {
                    // Tx "MOVE"  @NOTE: motors_ref done when the state machine switch between homing and trajectory after motor_ref will remain equal to old references
                    client->set_motors_references(MotorRefFlags::FLAG_MULTI_REF, motors_ref);
                    //->feed_motors(std::make_tuple(motor_ref_flags, motors_ref));
                }
                else
                {
                    throw std::runtime_error("fatal error: motors references structure empty!");
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
        
                if(STM_sts=="Engaged_Brake")
                {
                    // ************************* Engage Brake***********************************//
                    if(time_elapsed_ms>=time_to_engage_brakes)  
                    {
                        led_off_req=true;
                        if(motors_vel_check)
                        {
                            if(client->pdo_aux_cmd_sts(brake_cmds))
                            {
                                std::cout << "Brakes engaged for all motors" << std::endl;
                                stop_motors=true;
                            }
                            else
                            {
                                pdo_aux_cmd_attemps++;
                                if(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                                {
                                    led_off_req=false;
                                    motors_vel_check=false;
                                    start_time=time+period_ms_time;
                                }
                            }
                        }
                        else
                        {
                            std::cout << "Not all Motors velocity satisfied the velocity requirement"   << std::endl;  
                        }
                        
                        if(!stop_motors) // all motors velocity check not satisfied for time_to_engage_brakes or brake status different from request
                        {
                            std::cout << "Cannot engage the brake for all motors, no stopping action on the motors will be performed" << std::endl;  
                        }
                    }
                    else
                    {
                        if(!motors_vel_check)
                        {
                            brake_cmds.clear();
                            motors_vel_check=true;
                            for ( const auto &[esc_id, motor_velocity] : qdot){
                                
                                if(abs(motor_velocity) <= 7.0) // changed from 0.02 rad/s to 7rad/s same value of the slaves
                                {
                                    motors_vel_check &= true;
                                    brake_cmds.push_back(std::make_tuple(esc_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
                                }
                                else
                                {
                                    motors_vel_check &= false;
                                    std::cout << "Velocity check not satisfied for motor id: " << esc_id << std::endl;
                                }
                            }
                            
                            if(motors_vel_check)
                            {
                                start_time=time+period_ms_time;
                                if(!client->pdo_aux_cmd(brake_cmds))
                                { 
                                    run=false;
                                }
                            }
                        }
                    }
                    // ************************* Engage Brake***********************************//
                }
                    
                    
                    // ************************* SWITCH OFF LEDs OFF  ***********************************//
                if(STM_sts=="LED_OFF")
                {
                    if(time_elapsed_ms>=100ms) 
                    {
                        run=false;
                        
                        if(client->pdo_aux_cmd_sts(led_cmds))
                        {
                            std::cout << "Switched OFF the LEDs"<< std::endl;
                        }
                        else
                        {
                            pdo_aux_cmd_attemps++;
                            if(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                            {
                                run=true;
                                start_time=time+period_ms_time;
                                if(!client->pdo_aux_cmd(led_cmds))
                                {
                                    std::cout << "Cannot perform the led off command of the all motors" << std::endl;
                                    run=false;
                                }
                            }
                            else
                            {
                                std::cout << "Cannot swith off all leds" << std::endl; 
                            }
                        }
                    }
                    // ************************* SWITCH OFF LEDs  ***********************************//
                }
                
                // delay until time to iterate again
                time += period_ms_time;
                    
                if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing"))
                {
                    STM_sts="Trajectory";
                    start_time=time;
                    trajectory_counter=trajectory_counter+1;
                    set_trj_time_ms=trj_time_ms;
                    q_set_trj=ec_client_cfg.trajectory;
                    q_start=q_ref;
                }
                else if((time_elapsed_ms>=trj_time_ms)&&(STM_sts=="Trajectory"))
                {
                    if(trajectory_counter==ec_client_cfg.repeat_trj)
                    {
                        STM_sts="Engaged_Brake";
                        start_time=time;
                        pdo_aux_cmd_attemps=0;
                    }
                    else
                    {
                        STM_sts="Homing";
                        start_time=time;
                        set_trj_time_ms=hm_time_ms;
                        q_set_trj=ec_client_cfg.homing_position;
                        q_start=q_ref;
                    }
                }  
                else if(led_off_req && STM_sts=="Engaged_Brake")
                {
                    STM_sts="LED_OFF";
                    start_time=time;
                    
                    // SETUP LED OFF
                    led_cmds.clear();
                    for(int i=0; i<slave_id_vector.size();i++)
                    {
                        auto id=slave_id_vector[i];
                        for(int k=0 ; k < ec_client_cfg.slave_id_led.size();k++)  // led off only for the last slaves on the chain
                        {
                            if(id == ec_client_cfg.slave_id_led[k])
                            {
                                led_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::LED_OFF)));
                            }
                        }
                    }
                    
                    // send first leds off command
                    pdo_aux_cmd_attemps=0;
                    if(!client->pdo_aux_cmd(led_cmds))
                    {
                        std::cout << "Cannot perform the led off command of the all motors" << std::endl;
                        run=false;
                    }
                }
                    
                std::this_thread::sleep_until(time);             
            }
                
        }

        // ************************* STOP Motors ***********************************//
        if(stop_motors)
        {
            if(!client->stop_motors())
            {
                std::cout << "Not all motors are stopped" << std::endl;
            }
            else
            {
                std::cout << "All Motors stopped" << std::endl;
            }
                
        }
        // ************************* STOP Motors ***********************************//
        
        // STOP CLIENT
        if(client->is_client_alive())
        {
            client->stop_client();
        }
    }
    return 0;
}
