#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_common_step.h"
#include <test_common.h>

using namespace std::chrono;

int main(int argc, char * const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcCommonStep ec_common_step;
    
    try{
        ec_common_step.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }
    
    std::vector<int> motor_id_vector;
    for ( auto &[id, pos] : ec_cfg.homing_position ) {
        motor_id_vector.push_back(id);
    }
    
    if(motor_id_vector.empty()){
        DPRINTF("Got an homing position map\n");
        ec_common_step.stop_ec();
        return 1;
    }
    
    if(ec_cfg.trajectory.empty()){
        DPRINTF("Got an empty general trajectory map\n");
        ec_common_step.stop_ec();
        return 1;
    }
    
    bool motor_ctrl=false;
    try{
        ec_common_step.autodetection();
        motor_ctrl=ec_common_step.start_ec_motors(motor_id_vector);
        motor_ctrl=true;
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }


#ifdef TEST_EXAMPLES
    if(!motor_id_vector.empty())
    {
        motor_ctrl=true;
    }
#endif 

    if(motor_ctrl)
    {                                               
        struct timespec ts= { 0, ec_cfg.period_ms*1000000}; //sample time
        
        uint64_t start_time_ns=0;
        uint64_t time_ns=0;
        
        float time_elapsed_ms;
        float hm_time_ms=ec_cfg.homing_time_sec*1000;
        float trj_time_ms=ec_cfg.trajectory_time_sec*1000;
        float set_trj_time_ms=hm_time_ms;
        
        bool run=true;
        bool first_Rx=false;
        
        std::string STM_sts="Homing";
        std::map<int,double> q_set_trj=ec_cfg.homing_position;
        std::map<int,double> q_ref,q_start,qdot;

        
        int trajectory_counter=0;
        
        // Power Board
        PwrStatusMap pow_status_map;
        float v_batt,v_load,i_load,temp_pcb,temp_heatsink,temp_batt;
        
        // Motor
        float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
        float  motor_temp, board_temp;
        uint32_t fault,rtt,op_idx_ack;
        uint32_t cmd_aux_sts,brake_sts,led_sts;
        MotorStatusMap motors_status_map;
        
        float tau=0,alpha=0;
        MotorReferenceMap motors_ref;
        
        // memory allocation
        client->get_pow_status(pow_status_map);
        client->get_motors_status(motors_status_map);
        
        for ( const auto &[esc_id, motor_status] : motors_status_map){
            q_start[esc_id]= std::get<1>(motor_status); //motor pos
            qdot[esc_id] = std::get<3>(motor_status); //motor vel
            q_ref[esc_id]=q_start[esc_id];
        }
#ifdef TEST_EXAMPLES
        if(q_ref.empty()){
            for(int i=0; i<q_set_trj.size();i++){
                int id=motor_id_vector[i];
                q_start[id] = 0.0;
                qdot[id]    = 0.0;
                q_ref[id]   = 0.0;
            }
        }
#endif

        if(q_start.size() == q_set_trj.size()){
            //Open Loop SENSE
            first_Rx=true;
        }
        else{
            throw std::runtime_error("fatal error: different size of initial position and trajectory vectors");
        }
        
        for ( const auto &[esc_id, pos_ref] : q_ref){
           motors_ref[esc_id]=std::make_tuple(ec_cfg.motor_config_map[esc_id].control_mode_type, //ctrl_type
                                              pos_ref, //pos_ref
                                              0.0, //vel_ref
                                              0.0, //tor_ref
                                              ec_cfg.motor_config_map[esc_id].gains[0], //gain_1
                                              ec_cfg.motor_config_map[esc_id].gains[1], //gain_2
                                              ec_cfg.motor_config_map[esc_id].gains[2], //gain_3
                                              ec_cfg.motor_config_map[esc_id].gains[3], //gain_4
                                              ec_cfg.motor_config_map[esc_id].gains[4], //gain_5
                                              1, // op means NO_OP
                                              0, // idx
                                              0  // aux
                                              );
        }
        
        
        if(motors_ref.empty()){
            throw std::runtime_error("fatal error: motors references structure empty!");
        }
        
        if(motors_ref.size() != q_set_trj.size()){
            throw std::runtime_error("fatal error: different size of reference and trajectory vectors");
        }
        // memory allocation
                
        
        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            main_common (&argc, (char*const**)&argv, 0);
            assert(set_main_sched_policy(10) >= 0);
        }

        start_time_ns= iit::ecat::get_time_ns();
        time_ns=start_time_ns;
        
        while (run && client->is_client_alive())
        {
            time_elapsed_ms= (static_cast<float>((time_ns-start_time_ns))/1000000);
            //DPRINTF("Time [%f]\n",time_elapsed_ms);
            
            // Rx "SENSE"
            //******************* Power Board Telemetry ********
            client->get_pow_status(pow_status_map);
            for ( const auto &[esc_id, pow_rx_pdo] : pow_status_map){
                v_batt =        std::get<0>(pow_rx_pdo);
                v_load =        std::get<1>(pow_rx_pdo);
                i_load =        std::get<2>(pow_rx_pdo);
                temp_pcb =      std::get<3>(pow_rx_pdo);
                temp_heatsink=  std::get<4>(pow_rx_pdo);
                temp_batt=      std::get<5>(pow_rx_pdo);
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
                            DPRINTF("MOTOR ID: [%d], MOTOR_POS: [%f], LINK_POS: [%f]\n",esc_id,motor_pos,link_pos);
                        }
                } catch (std::out_of_range oor) {}
            }
            //******************* Motor Telemetry **************

            // define a simplistic linear trajectory
            tau= time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
            // interpolate
            for(int i=0; i<q_set_trj.size();i++)
            {
                int id=motor_id_vector[i];
                if(q_set_trj.count(id)>0)
                {
                    q_ref[id] = q_start[id] + alpha * (q_set_trj[id] - q_start[id]);
                }
            }
            
            // ************************* SEND ALWAYS REFERENCES***********************************//
            for ( const auto &[esc_id, pos_ref] : q_ref){
                std::get<1>(motors_ref[esc_id]) = pos_ref;
            }
            client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
            // ************************* SEND ALWAYS REFERENCES***********************************//

            if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing"))
            {
                STM_sts="Trajectory";
                start_time_ns=time_ns;
                trajectory_counter=trajectory_counter+1;
                set_trj_time_ms=trj_time_ms;
                q_set_trj=ec_cfg.trajectory;
                q_start=q_ref;
                tau=alpha=0;
            }
            else if((time_elapsed_ms>=trj_time_ms)&&(STM_sts=="Trajectory"))
            {
                if(trajectory_counter==ec_cfg.repeat_trj)
                {
                    start_time_ns=time_ns;
                    run=false;
                }
                else
                {
                    STM_sts="Homing";
                    start_time_ns=time_ns;
                    set_trj_time_ms=hm_time_ms;
                    q_set_trj=ec_cfg.homing_position;
                    q_start=q_ref;
                    tau=alpha=0;
                }
            } 
        
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
            // get period ns
            time_ns = iit::ecat::get_time_ns();
        }
            
    }
    
    ec_common_step.stop_ec_motors();
    ec_common_step.stop_ec();
    
    return 0;
}
