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
        ec_common_step.autodetection(motor_id_vector);
        //motor_ctrl=ec_common_step.start_ec_motors();
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
        
        uint64_t start_time_ns = iit::ecat::get_time_ns();
        uint64_t time_ns=start_time_ns;
        
        double time_elapsed_ms;
        double hm_time_ms=ec_cfg.homing_time_sec*1000;
        double trj_time_ms=ec_cfg.trajectory_time_sec*1000;
        double set_trj_time_ms=hm_time_ms;
        
        bool run=true;
        bool first_Rx=false;
        
        std::string STM_sts="Homing";
        std::map<int,double> q_set_trj=ec_cfg.homing_position;
        std::map<int,double> q_ref,q_start,qdot;
        
        double valve_curr_ref=5.0; //mA
        std::map<int,double> valves_trj_1,valves_trj_2,valves_set_zero,valves_set_trj;
        std::map<int,double> valves_set_ref,valves_start;

        
        int trajectory_counter=0;
        
        // Power Board
        PwrStatusMap pow_status_map;
        float v_batt,v_load,i_load,temp_pcb,temp_heatsink,temp_batt;
        
        // IMU
        ImuStatusMap imu_status_map;
        float x_rate,y_rate,z_rate;
        float x_acc,y_acc,z_acc;
        float x_quat,y_quat,z_quat,w_quat;
        
        // Valve
        ValveStatusMap valve_status_map;
        float encoder_position,tor_valve;           
        float pressure1,pressure2,temperature;   
        
        int valve_ref_index=0;
        std::vector<VR> valves_ref;
        
        
        // Motor
        float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
        float  motor_temp, board_temp;
        uint32_t fault,rtt,op_idx_ack;
        uint32_t cmd_aux_sts,brake_sts,led_sts;
        MotorStatusMap motors_status_map;
        
        double tau=0,alpha=0;
        std::vector<MR> motors_ref;
        int motor_ref_index=0;
        
        // memory allocation
        client->get_pow_status(pow_status_map);
        client->get_imu_status(imu_status_map);
        client->get_valve_status(valve_status_map);
        client->get_motors_status(motors_status_map);
        
        for ( const auto &[esc_id, valve_status] : valve_status_map){
            valves_trj_1[esc_id]=valve_curr_ref;
            valves_trj_2[esc_id]=-valve_curr_ref;
            valves_set_zero[esc_id]=0.0;
            valves_set_ref[esc_id]=valves_start[esc_id]=valves_set_zero[esc_id];
        }
        
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
        
        if(valves_set_ref.empty()){
            for(int i=0; i<ec_cfg.valve_id.size();i++){
                int valve_id=ec_cfg.valve_id[i];
                valves_trj_1[valve_id]=valve_curr_ref;
                valves_trj_2[valve_id]=-valve_curr_ref;
                valves_set_zero[valve_id]=0.0;
                valves_set_ref[valve_id]=valves_start[valve_id]=valves_set_zero[valve_id];
            }
        }
#endif

        valves_set_trj=valves_trj_1;
        //valves references check
        for ( const auto &[esc_id, curr_ref] : valves_set_ref){
            valves_ref.push_back(std::make_tuple(esc_id,curr_ref,0,0,0));
        }
        
        // motors references check
        if(!q_ref.empty()){
            if(q_start.size() == q_set_trj.size()){
                //Open Loop SENSE
                first_Rx=true;
            }
            else{
                throw std::runtime_error("fatal error: different size of initial position and trajectory vectors");
            }
            
            for ( const auto &[esc_id, pos_ref] : q_ref){
            motors_ref.push_back(std::make_tuple(esc_id, //bId
                                                    ec_cfg.motor_config_map[esc_id].control_mode_type, //ctrl_type
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
                                                ));
            }
            
            if(motors_ref.empty()){
                throw std::runtime_error("fatal error: motors references structure empty!");
            }
            
            if(motors_ref.size() != q_set_trj.size()){
                throw std::runtime_error("fatal error: different size of reference and trajectory vectors");
            }
        }
        
        if(motors_ref.empty() && valves_ref.empty()){
            throw std::runtime_error("fatal error: motor references and valves references are both empty");
        }
        // memory allocation
                
        
        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            main_common (&argc, (char*const**)&argv, 0);
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
                DPRINTF("POW ID: [%d], VBATT: [%f], VLOAD: [%f], ILOAD: [%f]\n",esc_id,v_batt,v_load,i_load);
            }
            //******************* Power Board Telemetry ********
            
            //******************* IMU Telemetry ********
            client->get_imu_status(imu_status_map);
            for ( const auto &[esc_id, imu_status] : imu_status_map){
                x_rate= imu_status[0];
                y_rate= imu_status[1];
                z_rate= imu_status[2];
                x_acc=  imu_status[3];
                y_acc=  imu_status[4];
                z_acc=  imu_status[5];
                x_quat= imu_status[6];
                y_quat= imu_status[7];
                z_quat= imu_status[8];
                w_quat= imu_status[9];
                DPRINTF("IMU ID: [%d], X_RATE: [%f], Y_RATE: [%f], Z_RATE: [%f]\n",esc_id,x_rate,y_rate,z_rate);
                DPRINTF("IMU ID: [%d], X_ACC: [%f], Y_ACC: [%f], Z_ACC: [%f]\n",esc_id,x_acc,y_acc,z_acc);
                DPRINTF("IMU ID: [%d], X_QUAT: [%f], Y_QUAT: [%f], Z_QUAT: [%f], W_QUAT: [%f]\n",esc_id,x_quat,y_quat,z_quat,w_quat);
            }
            //******************* IMU Telemetry ********
            
            //******************* Valve Telemetry ********
            client->get_valve_status(valve_status_map);
            for ( const auto &[esc_id, valve_status] : valve_status_map){
                encoder_position=   valve_status[0];
                tor_valve =         valve_status[1];
                pressure1 =         valve_status[2];
                pressure2 =         valve_status[3];
                temperature=        valve_status[4];
                DPRINTF("VALVE ID: [%d], Encoder pos: [%f], Torque: [%f]\n",esc_id,encoder_position,tor_valve);
                DPRINTF("VALVE ID: [%d], Press1: [%f], Press2: [%f],Temp: [%f]\n",esc_id,pressure1,pressure2,temperature);
            }
            //******************* Valve Telemetry ********
            
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
            
            // Valves references
            if(!valves_ref.empty()){
                // interpolate
                for ( const auto &[esc_id, target] : valves_set_trj){
                    valves_set_ref[esc_id] = valves_start[esc_id] + alpha * (target - valves_start[esc_id]);
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
                valve_ref_index=0;
                for ( const auto &[esc_id, curr_ref] : valves_set_ref){
                    std::get<1>(valves_ref[valve_ref_index]) = curr_ref;
                    valve_ref_index++;
                }
                client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }
            
            
            // Motors references
            if(!q_ref.empty()){
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
                motor_ref_index=0;
                for ( const auto &[esc_id, pos_ref] : q_ref){
                    std::get<2>(motors_ref[motor_ref_index]) = pos_ref;
                    motor_ref_index++;
                }
                client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // get period ns
            time_ns = iit::ecat::get_time_ns();
                
            if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing"))
            {
                STM_sts="Trajectory";
                start_time_ns=time_ns;
                set_trj_time_ms=trj_time_ms;
                
                q_set_trj=ec_cfg.trajectory;
                q_start=q_ref;
                
                if(trajectory_counter==ec_cfg.repeat_trj-1){ // second last references
                        valves_set_trj=valves_set_zero; //set to zero. (close valves)
                }
                else{
                    valves_set_trj=valves_trj_2;
                }
                
                valves_start=valves_set_ref;
                
                tau=alpha=0;
                trajectory_counter=trajectory_counter+1;
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
        
                    valves_set_trj=valves_trj_1;
                    valves_start=valves_set_ref;
                    
                    tau=alpha=0;
                }
            } 
            
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
        }
            
    }
    
    ec_common_step.stop_ec_motors();
    ec_common_step.stop_ec();
    
    return 0;
}
