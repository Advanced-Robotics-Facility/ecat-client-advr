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
    
    bool sys_ctrl=true;
    try{
        ec_common_step.autodetection();
        //sys_ctrl=ec_common_step.start_ec_motors(motor_id_vector);
        sys_ctrl &= ec_common_step.start_ec_valves(ec_cfg.valve_id);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }


#ifdef TEST_EXAMPLES
    sys_ctrl=true;
#endif 

    if(sys_ctrl)
    {        
        struct timespec ts= { 0, ec_cfg.period_ms*1000000}; //sample time
        
        uint64_t start_time_ns = iit::ecat::get_time_ns();
        uint64_t time_ns=start_time_ns;
        
        double time_elapsed_ms;
        double hm_time_ms=ec_cfg.homing_time_sec*1000;
        double trj_time_ms=ec_cfg.trajectory_time_sec*1000;
        double pressure_time_ms=1000; // 2minutes.
        double set_trj_time_ms=hm_time_ms;
        
        std::string STM_sts;
        bool run=true;
        bool first_motor_RX=false;
        
        std::map<int,double> q_set_trj=ec_cfg.homing_position;
        std::map<int,double> q_ref,q_start,qdot;
        
        bool first_pump_RX=false;
        uint8_t pump_pressure_ref=180; //bar
        std::map<int,uint8_t> pumps_trj_1,pumps_set_zero,pumps_set_trj;
        std::map<int,uint8_t> pumps_set_ref,pumps_start;
        
        double valve_curr_ref=2.5; //mA
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
        
        // Pump
        PumpStatusMap pump_status_map;
        PumpReferenceMap pumps_ref;
        
        // Valve
        ValveStatusMap valve_status_map;
        float encoder_position,tor_valve;           
        float pressure1,pressure2,temperature;   
        ValveReferenceMap valves_ref;
        
        // Motor
        float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
        float  motor_temp, board_temp;
        uint32_t fault,rtt,op_idx_ack;
        uint32_t cmd_aux_sts,brake_sts,led_sts;
        MotorStatusMap motors_status_map;
        
        double tau=0,alpha=0;
        MotorReferenceMap motors_ref;
        
        // memory allocation
        client->get_pow_status(pow_status_map);
        client->get_imu_status(imu_status_map);
        client->get_pump_status(pump_status_map);
        client->get_valve_status(valve_status_map);
        client->get_motors_status(motors_status_map);
        
        for ( const auto &[esc_id, pump_rx_pdo] : pump_status_map){
            pumps_trj_1[esc_id]=pump_pressure_ref;
            pumps_set_zero[esc_id]=0.0;
            pumps_set_ref[esc_id]=pumps_start[esc_id]=std::get<0>(pump_rx_pdo);
        }
        
        for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
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
        if(pumps_set_ref.empty()){
            for(int i=0; i<ec_cfg.pump_id.size();i++){
                int pump_id=ec_cfg.pump_id[i];
                pumps_trj_1[pump_id]=pump_pressure_ref;
                pumps_set_zero[pump_id]=0.0;
                pumps_set_ref[pump_id]=pumps_start[pump_id]=pumps_set_zero[pump_id];
            }
        }
#endif

        pumps_set_trj=pumps_trj_1;
        //pumps references check
        for ( const auto &[esc_id, press_ref] : pumps_set_trj){
            pumps_ref[esc_id]=std::make_tuple(press_ref,0,0,0,0,0,0,0,0);
        }

        valves_set_trj=valves_trj_1;
        //valves references check
        for ( const auto &[esc_id, curr_ref] : valves_set_ref){
            valves_ref[esc_id]=std::make_tuple(curr_ref,0,0,0,0,0,0,0);
        }
        
        // motors references check
        if(!q_ref.empty()){
            if(q_start.size() == q_set_trj.size()){
                //Open Loop SENSE
                first_motor_RX=true;
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
        }
        
        if(motors_ref.empty() && valves_ref.empty() && pumps_ref.empty()){
            throw std::runtime_error("fatal error: motor references, pump reference and valves references are both empty");
        }
        
        if(!pumps_ref.empty()){
            STM_sts="Pressure";
            set_trj_time_ms=pressure_time_ms;
        }
        else{
            STM_sts="Homing";
            set_trj_time_ms=hm_time_ms;
        }

        pumps_ref.clear();
        motors_ref.clear();
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
            for ( const auto &[esc_id, pow_rx_pdo] : pow_status_map){
                v_batt =        std::get<0>(pow_rx_pdo);
                v_load =        std::get<1>(pow_rx_pdo);
                i_load =        std::get<2>(pow_rx_pdo);
                temp_pcb =      std::get<3>(pow_rx_pdo);
                temp_heatsink=  std::get<4>(pow_rx_pdo);
                temp_batt=      std::get<5>(pow_rx_pdo);
                //DPRINTF("POW ID: [%d], VBATT: [%f], VLOAD: [%f], ILOAD: [%f]\n",esc_id,v_batt,v_load,i_load);
            }
            //******************* Power Board Telemetry ********
            
            //******************* IMU Telemetry ********
            client->get_imu_status(imu_status_map);
            for ( const auto &[esc_id, imu_rx_pdo] : imu_status_map){
                x_rate= std::get<0>(imu_rx_pdo);
                y_rate= std::get<1>(imu_rx_pdo);
                z_rate= std::get<2>(imu_rx_pdo);
                x_acc=  std::get<3>(imu_rx_pdo);
                y_acc=  std::get<4>(imu_rx_pdo);
                z_acc=  std::get<5>(imu_rx_pdo);
                x_quat= std::get<6>(imu_rx_pdo);
                y_quat= std::get<7>(imu_rx_pdo);
                z_quat= std::get<8>(imu_rx_pdo);
                w_quat= std::get<9>(imu_rx_pdo);
                //DPRINTF("IMU ID: [%d], X_RATE: [%f], Y_RATE: [%f], Z_RATE: [%f]\n",esc_id,x_rate,y_rate,z_rate);
                //DPRINTF("IMU ID: [%d], X_ACC: [%f], Y_ACC: [%f], Z_ACC: [%f]\n",esc_id,x_acc,y_acc,z_acc);
                //DPRINTF("IMU ID: [%d], X_QUAT: [%f], Y_QUAT: [%f], Z_QUAT: [%f], W_QUAT: [%f]\n",esc_id,x_quat,y_quat,z_quat,w_quat);
            }
            //******************* IMU Telemetry ********
            
            
            //******************* Pump Telemetry ********
            client->get_pump_status(pump_status_map);
            for ( const auto &[esc_id, pump_rx_pdo] : pump_status_map){
                //DPRINTF("PUMP ID: [%d], Pressure: [%hhu] \n",esc_id,std::get<0>(pump_rx_pdo));
                if(!first_pump_RX){
                    pumps_start[esc_id]=std::get<0>(pump_rx_pdo);
                    first_pump_RX=true;
                }
            }
            //******************* Pump Telemetry ********
            
            //******************* Valve Telemetry ********
            client->get_valve_status(valve_status_map);
            for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
                encoder_position=   std::get<0>(valve_rx_pdo);
                tor_valve =         std::get<1>(valve_rx_pdo);
                pressure1 =         std::get<2>(valve_rx_pdo);
                pressure2 =         std::get<3>(valve_rx_pdo);
                temperature=        std::get<4>(valve_rx_pdo);
                //DPRINTF("VALVE ID: [%d], Encoder pos: [%f], Torque: [%f]\n",esc_id,encoder_position,tor_valve);
                //DPRINTF("VALVE ID: [%d], Press1: [%f], Press2: [%f],Temp: [%f]\n",esc_id,pressure1,pressure2,temperature);
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
                            
                            if(!first_motor_RX)
                            {
                                q_start[esc_id]=motor_pos; // get actual motor position at first time
                            }
                            //DPRINTF("MOTOR ID: [%d], MOTOR_POS: [%f], LINK_POS: [%f]\n",esc_id,motor_pos,link_pos);
                        }
                } catch (std::out_of_range oor) {}
            }
            //******************* Motor Telemetry **************

            // define a simplistic linear trajectory
            tau= time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
            
            // Pump references
            if(!pumps_ref.empty()){
                if(STM_sts=="Pressure"){
                    // interpolate
                    for ( const auto &[esc_id, target] : pumps_set_trj){
                        pumps_set_ref[esc_id] = pumps_start[esc_id] + alpha * (target - pumps_start[esc_id]);
                    }
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
                for ( const auto &[esc_id, press_ref] : pumps_set_ref){
                    std::get<0>(pumps_ref[esc_id]) = press_ref;
                }
                client->set_pumps_references(RefFlags::FLAG_MULTI_REF, pumps_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }
            
            // Valves references
            if(!valves_ref.empty()){
                if(STM_sts!="Pressure"){
                    // interpolate
                    for ( const auto &[esc_id, target] : valves_set_trj){
                        valves_set_ref[esc_id] = valves_start[esc_id] + alpha * (target - valves_start[esc_id]);
                    }
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
                for ( const auto &[esc_id, curr_ref] : valves_set_ref){
                    std::get<0>(valves_ref[esc_id]) = curr_ref;
                }
                client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }
            
            
            // Motors references
            if(!motors_ref.empty()){
                if(STM_sts!="Pressure"){
                    // interpolate
                    for(int i=0; i<q_set_trj.size();i++)
                    {
                        int id=motor_id_vector[i];
                        if(q_set_trj.count(id)>0)
                        {
                            q_ref[id] = q_start[id] + alpha * (q_set_trj[id] - q_start[id]);
                        }
                    }
                }
                
                // ************************* SEND ALWAYS REFERENCES***********************************//
                for ( const auto &[esc_id, pos_ref] : q_ref){
                    std::get<1>(motors_ref[esc_id]) = pos_ref;
                }
                client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // get period ns
            time_ns = iit::ecat::get_time_ns();
            
            if((time_elapsed_ms>=pressure_time_ms)&&(STM_sts=="Pressure"))
            {
                if(trajectory_counter==ec_cfg.repeat_trj){
                    run=false;
                }
                else{
                    if(motors_ref.empty() && valves_ref.empty()){
                        STM_sts="Pressure";
                        start_time_ns=time_ns;
                        set_trj_time_ms=pressure_time_ms;
                        
                        pumps_set_trj=pumps_set_zero;
                        pumps_start=pumps_set_ref;
                        
                        tau=alpha=0;
                        trajectory_counter=ec_cfg.repeat_trj; // exit
                    }
                    else{
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
            }
            else if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing"))
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
                    if(!pumps_ref.empty()){
                        STM_sts="Pressure";
                        start_time_ns=time_ns;
                        set_trj_time_ms=pressure_time_ms;
                        
                        pumps_set_trj=pumps_set_zero;
                        pumps_start=pumps_set_ref;
                        
                        tau=alpha=0;
                    }
                    else{
                        run=false; // only homing or trajectory on valves/motors
                    }
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
    
    ec_common_step.stop_ec_valves();
    ec_common_step.stop_ec_motors();
    ec_common_step.stop_ec();
    
    return 0;
}
