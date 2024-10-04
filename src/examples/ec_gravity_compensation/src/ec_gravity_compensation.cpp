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
    if (STM_sts=="Motor_Ctrl_SetGains"){
        run_loop=false;
    }
    else if (STM_sts=="Motor_Ctrl"){
        restore_gains=true;
    }
}
                                                                    
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
    
    if(ec_cfg.homing_position.empty()){
        DPRINTF("Got an homing position map\n");
        return 1;
    }
    
    bool ec_sys_started = true;
    try{
        ec_sys_started = ec_common_step.start_ec_sys();
        if(ec_sys_started){
            STM_sts="Motor_Ctrl_SetGains";
        }
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }
         
    if(STM_sts=="Motor_Ctrl_SetGains") {
        // MODEL INTEFACE SETUP
        auto ec_cfg_file=ec_common_step.get_ec_utils()->get_ec_cfg_file();
        XBot::ConfigOptions config=XBot::ConfigOptions::FromConfigFile(ec_cfg_file); 
        
        XBot::ModelInterface::Ptr model= XBot::ModelInterface::getModel(config);
        
        if(model==nullptr){
            DPRINTF("fatal error: Got an empty model\n");
            ec_common_step.stop_ec_sys();
            return 1;
        }
        
        float time_elapsed_ms;
        float tau=0,alpha=0;

        XBot::JointIdMap q,q_ref,tau_ref;
        Eigen::VectorXd tau_g; 
        
        // memory allocation
        std::map<int,std::vector<float>> imp_gains_map,imp_zero_gains_map;
        for (const auto &[esc_id, motor_rx_pdo] : motors_status_map){
            if(ec_cfg.homing_position.count(esc_id)>0){
                imp_gains_map[esc_id]=ec_cfg.motor_config_map[esc_id].gains;
                imp_zero_gains_map[esc_id]=imp_gains_map[esc_id];
                imp_zero_gains_map[esc_id][0]=0.0;
                imp_zero_gains_map[esc_id][1]=0.0;
                q[esc_id]=std::get<1>(motor_rx_pdo); //motor pos
                q_ref[esc_id]=q[esc_id];
            }
        }

        if(q_ref.empty()){
            throw std::runtime_error("fatal error: motors references structure empty!");
        }

        size_t join_num= static_cast<size_t>(model->getJointNum());
        if(model->isFloatingBase()){
            join_num=join_num-6;
        }
        if(q.size() != join_num){
            throw std::runtime_error("fatal error: different size of initial position from joint of the model");
        }

        float set_gain_time_ms=3000; // Default 3s 
        std::map<int,std::vector<float>> gain_start_map=imp_gains_map;
        std::map<int,std::vector<float>> gain_trj_map=imp_zero_gains_map;
        std::map<int,std::vector<float>> gain_ref_map=gain_start_map;        
        // memory allocation
    
        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            signal ( SIGALRM, sig_handler );
            main_common (&argc, (char*const**)&argv, sig_handler);
            int priority = SCHED_OTHER;
            #if defined(PREEMPT_RT) || defined(__COBALT__)
                priority = sched_get_priority_max ( SCHED_FIFO ) / 3;
            #endif
            int ret = set_main_sched_policy(priority);
            if (ret < 0){
                throw std::runtime_error("fatal error on set_main_sched_policy");
            }
        }
        else{
            struct sigaction sa;
            sa.sa_handler = sig_handler;
            sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
            sigaction(SIGINT,&sa, nullptr);
        }
    
        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);
        
        while (run_loop && client->get_client_status().run_loop){
            client->read();
            ec_common_step.telemetry();

            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Time elapsed ms: [%f]\n",time_elapsed_ms);
                
            if(STM_sts=="Motor_Ctrl_SetGains"){
                // define a simplistic linear trajectory
                tau= time_elapsed_ms / set_gain_time_ms;

                // quintic poly 6t^5 - 15t^4 + 10t^3
                alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
                
                // interpolate
                for ( auto &[esc_id, gain_ref] : gain_ref_map){
                    uint8_t i=0;
                    for(auto &gain_ref_value:gain_ref){
                        gain_ref_value = gain_start_map[esc_id][i] + alpha * (gain_trj_map[esc_id][i] - gain_start_map[esc_id][i]);
                        i++;
                    }
                }
            }

            for (const auto &[esc_id, motor_rx_pdo] : motors_status_map){
                if(ec_cfg.homing_position.count(esc_id)>0){
                    q[esc_id]=std::get<1>(motor_rx_pdo); //motor pos
                    q_ref[esc_id]=q[esc_id];
                }
            }   

            model->setJointPosition(q);
            model->update();
            
            model->computeGravityCompensation(tau_g);
            model->eigenToMap(tau_g,tau_ref);

            for ( const auto &[esc_id, tor_ref] : tau_ref){
                if(ec_cfg.homing_position.count(esc_id)>0){
                    double pos_ref = q_ref[esc_id];
                    std::get<1>(motors_ref[esc_id]) = pos_ref;
                    std::get<3>(motors_ref[esc_id]) = tor_ref;
                    std::get<4>(motors_ref[esc_id]) = gain_ref_map[esc_id][0];
                    std::get<5>(motors_ref[esc_id]) = gain_ref_map[esc_id][1];
                    std::get<6>(motors_ref[esc_id]) = gain_ref_map[esc_id][2];
                    std::get<7>(motors_ref[esc_id]) = gain_ref_map[esc_id][3];
                    std::get<8>(motors_ref[esc_id]) = gain_ref_map[esc_id][4];
                }
            }
            
            // ************************* SEND ALWAYS REFERENCES***********************************//
            client->set_motors_references(motors_ref);
            // ************************* SEND ALWAYS REFERENCES***********************************//

            time = time + period;
            
            if(STM_sts=="Motor_Ctrl_SetGains"){
                if(time_elapsed_ms>=set_gain_time_ms){
                    STM_sts="Motor_Ctrl";
                    start_time=time;
                }
            }
            else if(STM_sts=="Motor_Ctrl")
            {
                if(restore_gains){
                    STM_sts="Motor_Ctrl_SetGains";
                    gain_start_map=gain_ref_map;
                    gain_trj_map=imp_gains_map;
                    start_time=time;
                }
            }
            
            client->write();
            client->log();
            
            const auto now = std::chrono::high_resolution_clock::now();

#if defined(PREEMPT_RT) || defined(__COBALT__)
            // if less than threshold, print warning (only on rt threads)
            if (now > time && ec_cfg.protocol == "iddp"){
                ++overruns;
                DPRINTF("main process overruns: %d\n", overruns);
            }
#endif
            std::this_thread::sleep_until(time);
        }
            
    }
    
    ec_common_step.stop_ec_sys();
    
    return 0;
}
