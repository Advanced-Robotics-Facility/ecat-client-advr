#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_utils.h"
#include <test_common.h>
#include <utils.h>

using namespace std::chrono;

int main()
{
    EcUtils::Ptr ec_client_utils;
    EcUtils::EC_CONFIG ec_client_cfg;
    
    try{
        ec_client_utils=std::make_shared<EcUtils>();
        ec_client_cfg = ec_client_utils->get_ec_cfg();
    }catch(std::exception &ex){
        DPRINTF("Error on ec client config file\n");
        DPRINTF("%s\n",ex.what());
    return 1;
    }

    std::vector<int> slave_id_vector;
    for ( auto &[id, pos] : ec_client_cfg.homing_position ) {
        slave_id_vector.push_back(id);
    }
    
    if(slave_id_vector.empty()){
        DPRINTF("Got an homing position map\n");
        return 0;
    }
    
    if(ec_client_cfg.trajectory.empty()){
        DPRINTF("Got an empty general trajectory map\n");
        return 0;
    }

    // *************** START CLIENT  *************** //
    EcIface::Ptr client=ec_client_utils->make_ec_iface();
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
            DPRINTF("AUTODETECTION\n");
            for(int motor_id_index=0;motor_id_index<slave_id_vector.size();motor_id_index++)
            {
                bool motor_found=false;
                for ( auto &[id, type, pos] : slave_info ) {
                    if((type==CENT_AC) || (type==LO_PWR_DC_MC))//HP or LP motor
                    {
                        if(id == ec_client_cfg.motor_id[motor_id_index])
                        {
                            motor_found=true;
                            break;
                        }
                    }
                }
            
                if(!motor_found)
                {
                    throw std::runtime_error("ID: " + std::to_string(ec_client_cfg.motor_id[motor_id_index]) + "not found");
                }
            }

        }
    }
    
    for(int i=0; i< slave_id_vector.size();i++)
    {
        int id = slave_id_vector[i];
        motors_start.push_back(std::make_tuple(id,ec_client_cfg.motor_config_map[id].control_mode_type,ec_client_cfg.motor_config_map[id].gains));
        
        if(ec_client_cfg.motor_config_map[id].brake_present){
            // queue release brake commands for all motors 
            brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
        }
        
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
        DPRINTF("START ALL MOTORS\n");
        motor_started=client->start_motors(motors_start);

        if(motor_started)
        {
            if(!brake_cmds.empty()){
                // ************************* RELEASE BRAKES ***********************************//
                while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                {
                    pdo_aux_cmd_attemps++;
                    if(!client->pdo_aux_cmd(brake_cmds))
                    {
                        DPRINTF("Cannot perform the release brake command of the motors\n");
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
            else{
                send_ref=true;
            }
            
        }
        else
        {
            DPRINTF("Motors not started\n");
        }
            
        // ************************* START Motors ***********************************//

        if(!led_cmds.empty()){
            // ************************* SWITCH ON LEDs ***********************************//
            pdo_aux_cmd_attemps=0;
            while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
            {
                pdo_aux_cmd_attemps++;
                if(!client->pdo_aux_cmd(led_cmds))
                {
                    DPRINTF("Cannot perform the led on command of the motors\n");
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                }
                else
                {
                    std::this_thread::sleep_for(100ms);
                    if(client->pdo_aux_cmd_sts(led_cmds))
                    {
                        DPRINTF("Switched ON the LEDs\n");
                        pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    }
                }
            }
            // ************************* SWITCH ON LEDs ***********************************//
        }
    }
    else
    {
        DPRINTF("NO MOTORS STARTED\n");
    }

#ifdef TEST_EXAMPLES
    if(!slave_id_vector.empty())
    {
        send_ref=true;
    }
#endif
            
    if(send_ref)
    {                                               
        struct timespec ts= { 0, ec_client_cfg.period_ms*1000000}; //sample time
        
        uint64_t period_ns=ec_client_cfg.period_ms*1000000;
        uint64_t start_time_ns = iit::ecat::get_time_ns();
        uint64_t time_ns=start_time_ns;
        
        double time_elapsed_ms;
        double hm_time_ms=ec_client_cfg.homing_time_sec*1000;
        double trj_time_ms=ec_client_cfg.trajectory_time_sec*1000;
        double set_trj_time_ms=hm_time_ms;
        double time_to_engage_brakes_ms = 1000; // Default 1s
        
        bool run=true;
        bool first_Rx=false;
        
        std::string STM_sts="Homing";
        std::map<int,double> q_set_trj=ec_client_cfg.homing_position;
        std::map<int,double> q_ref,q_start,qdot;

        
        int trajectory_counter=0;
        bool motors_vel_check=false;
        bool led_off_req=false;
        
        // Power Board
        PwrStatusMap pow_status_map;
        float v_batt,v_load,i_load,temp_pcb,temp_heatsink,temp_batt;
        
        // Motor
        float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
        float  motor_temp, board_temp;
        uint32_t fault,rtt,op_idx_ack;
        uint32_t cmd_aux_sts,brake_sts,led_sts;
        MotorStatusMap motors_status_map;
        
        double tau=0,alpha=0;
        std::vector<MR> motors_ref;
        
        // pre-allocation for rt code
        client->get_pow_status(pow_status_map);
        client->get_motors_status(motors_status_map);
        
        if(ec_client_cfg.protocol=="iddp")
        {
            DPRINTF("Real-time process....\n");
            assert(set_main_sched_policy(10) >= 0);
        }
    
        while (run && client->is_client_alive())
        {
            time_elapsed_ms= (time_ns-start_time_ns)/1000000;
            //DPRINTF("Time [%f]\n",time_elapsed_ms);
            
            // Rx "SENSE"
            //******************* Power Board Telemetry ********
            client->get_pow_status(pow_status_map);
            for ( const auto &[esc_id, pow_status] : pow_status_map){
                v_batt =        pow_status[0];
                v_load =        pow_status[1];
                i_load =        pow_status[2];
                temp_pcb =      pow_status[3];
                temp_heatsink=  pow_status[4];
                temp_batt=      pow_status[5];
            }

            //******************* Power Board Telemetry ********
            
            //******************* Motor Telemetry **************
            client->get_motors_status(motors_status_map);
            for ( const auto &[esc_id, motor_status] : motors_status_map){
                try {
                        if(q_set_trj.count(esc_id))
                        {
                            std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = motor_status;
                            
                            // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                            brake_sts = cmd_aux_sts & 3; //00 unknown
                                                        //01 release brake 
                                                        //10 enganged brake  
                                                        //11 error
                            led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                            
                            //Closed Loop SENSE for motor velocity
                            qdot[esc_id] = motor_vel;
                            
                            if(!first_Rx)
                            {
                                q_start[esc_id]=motor_pos; // get actual motor position at first time
                            }
                        }
                } catch (std::out_of_range oor) {}
            }
            //******************* Motor Telemetry **************

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
            if(q_start.size() == q_set_trj.size())
            {
                //Open Loop SENSE
                first_Rx=true;
            }
            else
            {
                throw std::runtime_error("fatal error: different size of initial position and trajectory vectors");
            }
                
            if((STM_sts=="Homing")||
                (STM_sts=="Trajectory"))
            {
                motors_ref.clear();  //clear old trajectory
                // define a simplistic linear trajectory
                tau= time_elapsed_ms / set_trj_time_ms;
                // quintic poly 6t^5 - 15t^4 + 10t^3
                alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
                // interpolate
                for(int i=0; i<q_set_trj.size();i++)
                {
                    int id=slave_id_vector[i];
                    if(q_set_trj.count(id)>0)
                    {
                        q_ref[id] = q_start[id] + alpha * (q_set_trj[id] - q_start[id]);
                    }
                }
                for(int i=0; i<slave_id_vector.size();i++)
                {
                    auto id=slave_id_vector[i];
                    if(q_ref.count(id)>0)
                    {
                        auto pos= q_ref[id];
                        
                        motors_ref.push_back(std::make_tuple(id, //bId
                                                                ec_client_cfg.motor_config_map[id].control_mode_type, //ctrl_type
                                                                pos, //pos_ref
                                                                0.0, //vel_ref
                                                                0.0, //tor_ref
                                                                ec_client_cfg.motor_config_map[id].gains[0], //gain_1
                                                                ec_client_cfg.motor_config_map[id].gains[1], //gain_2
                                                                ec_client_cfg.motor_config_map[id].gains[2], //gain_3
                                                                ec_client_cfg.motor_config_map[id].gains[3], //gain_4
                                                                ec_client_cfg.motor_config_map[id].gains[4], //gain_5
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
                client->set_motors_references(MotorRefFlags::FLAG_MULTI_REF, motors_ref);
            }
            else
            {
                throw std::runtime_error("fatal error: motors references structure empty!");
            }
            
            // ************************* SEND ALWAYS REFERENCES***********************************//
    
            if(STM_sts=="Engaged_Brake")
            {
                // ************************* Engage Brake***********************************//
                if(time_elapsed_ms>=time_to_engage_brakes_ms)  
                {
                    led_off_req=true;
                    if(motors_vel_check)
                    {
                        if(client->pdo_aux_cmd_sts(brake_cmds))
                        {
                            DPRINTF("Brakes engaged for all motors\n");
                            stop_motors=true;
                        }
                        else
                        {
                            pdo_aux_cmd_attemps++;
                            if(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                            {
                                led_off_req=false;
                                motors_vel_check=false;
                                start_time_ns=time_ns+period_ns;
                            }
                        }
                    }
                    else
                    {
                        DPRINTF("Not all Motors velocity satisfied the velocity requirement\n");  
                    }
                    
                    if(!stop_motors) // all motors velocity check not satisfied for time_to_engage_brakes or brake status different from request
                    {
                        DPRINTF("Cannot engage the brake for all motors, no stopping action on the motors will be performed\n");
                    }
                }
                else
                {
                    if(!motors_vel_check)
                    {
                        brake_cmds.clear();
                        motors_vel_check=true;
                        for ( const auto &[esc_id, motor_velocity] : qdot){
                            if(ec_client_cfg.motor_config_map[esc_id].brake_present){
                                if(abs(motor_velocity) <= 7.0) // changed from 0.02 rad/s to 7rad/s same value of the slaves
                                {
                                    motors_vel_check &= true;
                                    brake_cmds.push_back(std::make_tuple(esc_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
                                }
                                else
                                {
                                    motors_vel_check &= false;
                                    DPRINTF("Velocity check not satisfied for motor id: %d\n", esc_id);
                                }
                            }
                        }
                        
                        if(motors_vel_check)
                        {
                            if(!brake_cmds.empty()){
                                start_time_ns=time_ns+period_ns;
                                if(!client->pdo_aux_cmd(brake_cmds))
                                { 
                                    run=false;
                                }
                            }
                            else{
                                led_off_req=true;
                                stop_motors=true;
                            }
                        }
                    }
                }
                // ************************* Engage Brake***********************************//
            }
                
                
                // ************************* SWITCH OFF LEDs OFF  ***********************************//
            if(STM_sts=="LED_OFF")
            {
                if(time_elapsed_ms>=100) 
                {
                    run=false;
                    if(!led_cmds.empty()){
                        if(client->pdo_aux_cmd_sts(led_cmds))
                        {
                            DPRINTF("Switched OFF the LEDs\n");
                        }
                        else
                        {
                            pdo_aux_cmd_attemps++;
                            if(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                            {
                                run=true;
                                start_time_ns=time_ns+period_ns;
                                if(!client->pdo_aux_cmd(led_cmds))
                                {
                                    DPRINTF("Cannot perform the led off command of the all motors\n");
                                    run=false;
                                }
                            }
                            else
                            {
                                DPRINTF("Cannot swith off all leds\n"); 
                            }
                        }
                    }
                }
                // ************************* SWITCH OFF LEDs  ***********************************//
            }
            
            // get period ns
            time_ns = iit::ecat::get_time_ns();
                
            if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing"))
            {
                STM_sts="Trajectory";
                start_time_ns=time_ns;
                trajectory_counter=trajectory_counter+1;
                set_trj_time_ms=trj_time_ms;
                q_set_trj=ec_client_cfg.trajectory;
                q_start=q_ref;
                tau=alpha=0;
            }
            else if((time_elapsed_ms>=trj_time_ms)&&(STM_sts=="Trajectory"))
            {
                if(trajectory_counter==ec_client_cfg.repeat_trj)
                {
                    STM_sts="Engaged_Brake";
                    start_time_ns=time_ns;
                    pdo_aux_cmd_attemps=0;
                }
                else
                {
                    STM_sts="Homing";
                    start_time_ns=time_ns;
                    set_trj_time_ms=hm_time_ms;
                    q_set_trj=ec_client_cfg.homing_position;
                    q_start=q_ref;
                    tau=alpha=0;
                }
            }  
            else if(led_off_req && STM_sts=="Engaged_Brake")
            {
                STM_sts="LED_OFF";
                start_time_ns=time_ns;
                
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
                    DPRINTF("Cannot perform the led off command of the all motors\n");
                    run=false;
                }
            }
            clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        }
            
    }

    // ************************* STOP Motors ***********************************//
    if(stop_motors)
    {
        if(!client->stop_motors())
        {
            DPRINTF("Not all motors are stopped\n");
        }
        else
        {
            DPRINTF("All Motors stopped\n");
        }
            
    }
    // ************************* STOP Motors ***********************************//
    
    // STOP CLIENT
    if(client->is_client_alive())
    {
        client->stop_client();
    }
    return 0;
}
