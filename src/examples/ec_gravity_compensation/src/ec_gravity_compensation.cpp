#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include <atomic>

#include "utils/ec_common_step.h"
#include <test_common.h>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/ConfigOptions.h>


using namespace std::chrono;

std::string STM_sts="IDLE";
bool restore_gains=false;


static bool run_loop = true;
// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    if (STM_sts=="Motor_Ctrl_SetGains")
    {
        run_loop=false;
    }
    else if (STM_sts=="Motor_Ctrl")
    {
        restore_gains=true;
    }
}

// IDLE-->Connected->Autodetection->Motor_Started->Motor_Ctrl_SetGains->Motor_Ctrl->Motor_Ctrl_SetGains->Engage_Motor_Brake->Motor_Stopping->Exit
                                                                    
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
    
    bool motor_ctrl=false;
    try{
        ec_common_step.autodetection();
        motor_ctrl=ec_common_step.start_ec_motors(motor_id_vector);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }
    
#ifdef TEST_EXAMPLES
    if(!motor_id_vector.empty())
    {
        STM_sts="Motor_Ctrl_SetGains";
    }
#endif
            
    if(STM_sts=="Motor_Ctrl_SetGains")
    {
        // MODEL INTEFACE SETUP
        auto ec_cfg_file=ec_common_step.get_ec_utils()->get_ec_cfg_file();
        XBot::ConfigOptions config=XBot::ConfigOptions::FromConfigFile(ec_cfg_file); 
        
        XBot::ModelInterface::Ptr model= XBot::ModelInterface::getModel(config);
        
        if(model==nullptr){
            DPRINTF("fatal error: Got an empty model\n");
            ec_common_step.stop_ec();
            return 1;
        }
        
        struct timespec ts= { 0, ec_cfg.period_ms*1000000}; //sample time
        
        uint64_t start_time_ns = iit::ecat::get_time_ns();
        uint64_t time_ns=start_time_ns;
        
        float time_elapsed_ms;
        
        bool first_Rx=false;

        
        XBot::JointIdMap q,qdot,q_ref,tau_ref;
        Eigen::VectorXd tau_g; 
        
        
        /// GAINS
        std::map<int,std::vector<float>> imp_gains_map,imp_zero_gains_map;
        for(int i=0; i< motor_id_vector.size();i++)
        {
            int id = motor_id_vector[i];
            imp_gains_map[id]=ec_cfg.motor_config_map[id].gains;
            imp_zero_gains_map[id]=imp_gains_map[id];
            imp_zero_gains_map[id][0]=0.0;
            imp_zero_gains_map[id][1]=0.0;
            imp_zero_gains_map[id][2]=0.0;
        }

        float set_gain_time_ms=3000; // Default 3s 
        std::map<int,std::vector<float>> gain_start_map=imp_gains_map;
        std::map<int,std::vector<float>> gain_trj_map=imp_zero_gains_map;
        std::map<int,std::vector<float>> gain_ref_map=gain_start_map;
        /// GAINS
        
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
            q[esc_id]=std::get<1>(motor_status); //motor pos
            qdot[esc_id] = std::get<3>(motor_status); //motor vel
            q_ref[esc_id]=q[esc_id];
        }

#ifdef TEST_EXAMPLES
        if(q_ref.empty()){
            for(int i=0; i<motor_id_vector.size();i++){
                int id=motor_id_vector[i];
                q[id]=0.0;
                qdot[id] = 0.0;
                q_ref[id]=0.0;
            }
        }
#endif

        if(q.size() == model->getJointNum()-6){
            //Open Loop SENSE
            first_Rx=true;
        }
        else{
            throw std::runtime_error("fatal error: different size of initial position from joint of the model");
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
        
        // memory allocation
    
        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            signal ( SIGALRM, sig_handler );
            main_common (&argc, (char*const**)&argv, sig_handler);
            assert(set_main_sched_policy(10) >= 0);
        }
        else{
            struct sigaction sa;
            sa.sa_handler = sig_handler;
            sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
            sigaction(SIGINT,&sa, nullptr);
        }
    
        
        while (run_loop && client->is_client_alive())
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
                try{
                    std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = motor_status;
                    
                    // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                    brake_sts = cmd_aux_sts & 3; //00 unknown
                                                //01 release brake 
                                                //10 enganged brake  
                                                //11 error
                    led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                    
                    
                    //Closed Loop SENSE for motor position and velocity
                    q[esc_id]=motor_pos;
                    qdot[esc_id] = motor_vel;
                    q_ref[esc_id]=q[esc_id];
                    
                } catch (std::out_of_range oor) {}
            }

            //******************* Motor Telemetry **************
                
            if(STM_sts=="Motor_Ctrl_SetGains")
            {
                // define a simplistic linear trajectory
                tau= time_elapsed_ms / set_gain_time_ms;

                // quintic poly 6t^5 - 15t^4 + 10t^3
                alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
                
                // interpolate
                for ( auto &[esc_id, gain_ref] : gain_ref_map){
                    for(int i=0;i<gain_ref.size();i++){
                        gain_ref[i] = gain_start_map[esc_id][i] + alpha * (gain_trj_map[esc_id][i] - gain_start_map[esc_id][i]);
                    }
                }
            }

            model->setJointPosition(q);
            model->update();
            
            model->computeGravityCompensation(tau_g);
            model->eigenToMap(tau_g,tau_ref);
            
            // ************************* SEND ALWAYS REFERENCES***********************************//
            for ( const auto &[esc_id, tor_ref] : tau_ref){
                double pos_ref = q_ref[esc_id];
                std::get<1>(motors_ref[esc_id]) = pos_ref;
                std::get<3>(motors_ref[esc_id]) = tor_ref;
                std::get<4>(motors_ref[esc_id]) = gain_ref_map[esc_id][0];
                std::get<5>(motors_ref[esc_id]) = gain_ref_map[esc_id][1];
                std::get<6>(motors_ref[esc_id]) = gain_ref_map[esc_id][2];
                std::get<7>(motors_ref[esc_id]) = gain_ref_map[esc_id][3];
                std::get<8>(motors_ref[esc_id]) = gain_ref_map[esc_id][4];
            }
            client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
            // ************************* SEND ALWAYS REFERENCES***********************************//
            
            // get period ns
            time_ns = iit::ecat::get_time_ns();
            
            if(STM_sts=="Motor_Ctrl_SetGains")
            {
                if(time_elapsed_ms>=set_gain_time_ms)
                {
                    STM_sts="Motor_Ctrl";
                    start_time_ns=time_ns;
                }
            }
            else if(STM_sts=="Motor_Ctrl")
            {
                if(restore_gains)
                {
                    STM_sts="Motor_Ctrl_SetGains";
                    gain_start_map=gain_ref_map;
                    gain_trj_map=imp_gains_map;
                    start_time_ns=time_ns;
                }
            }
            
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
        }
            
    }
    
    ec_common_step.stop_ec_motors();
    ec_common_step.stop_ec();
    
    return 0;
}
