#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_wrapper.h"
#include <test_common.h>

using namespace std::chrono;

static bool run_loop = true;
// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    run_loop=false;
}

int main(int argc, char * const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcWrapper ec_wrapper;
    std::string fatal_error="";
    try{
        ec_wrapper.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }

    std::map<int,double> moto_homing,grp_homing;
    if(ec_cfg.trj_config_map.count("motor")>0){
        moto_homing=ec_cfg.trj_config_map["motor"].homing;    
        grp_homing=ec_cfg.trj_config_map["gripper"].trajectory;    
    }

    if(moto_homing.empty() && grp_homing.empty()){
        DPRINTF("Got an empty homing position map\n");
        return 1;
    }

    bool ec_sys_started = true;
    try{
        ec_sys_started = ec_wrapper.start_ec_sys();
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }

    if(ec_sys_started){                       
        int overruns = 0;
        float time_elapsed_ms, sample_time_ms;
        float hm_time_ms = ec_cfg.trj_time * 1000;
        float trj_time_ms = hm_time_ms;
        float set_trj_time_ms = hm_time_ms;
        
        std::string STM_sts="Homing";

        int trajectory_counter=0;
        float tau=0,alpha=0;

        if(motor_trj_map.empty() && gripper_trj_map.empty()){
            fatal_error="fatal error: devices references structure empty!";
            run_loop=false;
        }else{           
            if (ec_cfg.protocol == "iddp"){
                // add SIGALRM
                signal ( SIGALRM, sig_handler );
                //avoid map swap
                main_common (&argc, (char*const**)&argv, sig_handler);
            }
            else{
                struct sigaction sa;
                sa.sa_handler = sig_handler;
                sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
                sigaction(SIGINT,&sa, nullptr);
            }
            
            // process scheduling
            try{
                ec_wrapper.ec_self_sched(argv[0]);
            }catch(std::exception& e){
                std::string error=e.what();
                fatal_error="fatal error: "+ error;
                run_loop=false;
            }
        }

        using Clock = std::chrono::steady_clock;
        using us = std::chrono::microseconds;

        auto start_time = Clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);

        auto to_us = [](auto duration) -> int64_t {
            return std::chrono::duration_cast<us>(duration).count();
        };
        
        while (run_loop && client->get_client_status().run_loop){
            const auto scheduled_release = time;
            const auto t0 = Clock::now();
            client->read();
            const auto t1 = Clock::now();
            
            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Time elapsed ms: [%f]\n",time_elapsed_ms);
        
            // define a simplistic linear trajectory
            tau= time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
            alpha = std::clamp(alpha, 0.0f, 1.0f);
            // interpolate
            for (auto &[esc_id, motor_trj] : motor_trj_map){
                int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                motor_trj.set_target(motor_trj.start + alpha * (motor_trj.set_trj - motor_trj.start));
                if(ctrl_mode != iit::advr::Gains_Type_VELOCITY){
                    if(ctrl_mode == iit::advr::Gains_Type_POSITION ||
                    ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                        std::get<1>(motor_reference_map[esc_id]) = motor_trj.set_ref;
                    }
                    if(ctrl_mode != iit::advr::Gains_Type_POSITION &&
                    ctrl_mode != iit::advr::Gains_Type_IMPEDANCE){
                        std::get<3>(motor_reference_map[esc_id]) = motor_trj.set_ref; // current mode (0xCC or oxDD) or impedance
                    }
                }else{
                    std::get<2>(motor_reference_map[esc_id]) = motor_trj.set_ref;
                }
            }  
            // ************************* SEND ALWAYS REFERENCES***********************************//
            if(!motor_trj_map.empty()){
                client->set_motor_reference(motor_reference_map);
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//

            for (auto &[esc_id, gripper_trj] : gripper_trj_map){
                int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                gripper_trj.set_target(gripper_trj.start + alpha * (gripper_trj.set_trj - gripper_trj.start));
                if(ctrl_mode == iit::advr::Gains_Type_POSITION){
                    std::get<1>(gripper_reference_map[esc_id]) = gripper_trj.set_ref;
                }
            }  
            // ************************* SEND ALWAYS REFERENCES***********************************//
            if(!gripper_trj_map.empty()){
                client->set_gripper_reference(gripper_reference_map);
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//

            const auto t2 = Clock::now();

            time = time + period;

            if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing")) {
                STM_sts = "Trajectory";
                start_time = time;
                set_trj_time_ms = trj_time_ms;

                if (trajectory_counter == ec_cfg.repeat_trj - 1){
                    set_esc_trj(motor_trj_map,TrjType::zero);
                    set_esc_trj(gripper_trj_map,TrjType::zero);

                }else{
                    set_esc_trj(motor_trj_map,TrjType::trj2);
                    set_esc_trj(gripper_trj_map,TrjType::trj2);
                }

                tau = alpha = 0;
                trajectory_counter = trajectory_counter + 1;

            }
            else if((time_elapsed_ms>=trj_time_ms)&&(STM_sts=="Trajectory")){
                if(trajectory_counter==ec_cfg.repeat_trj){
                    start_time=time;
                    run_loop=false;
                }
                else{
                    STM_sts = "Homing";
                    start_time = time;
                    set_trj_time_ms = hm_time_ms;
                    
                    set_esc_trj(motor_trj_map,TrjType::trj1);
                    set_esc_trj(gripper_trj_map,TrjType::trj1);

                    tau = alpha = 0;
                }
            } 
            const auto t3 = Clock::now();
            client->write();
            const auto t4 = Clock::now();
            ec_wrapper.log_ec_sys();
            const auto end = Clock::now();

#if defined(PREEMPT_RT) || defined(__COBALT__)
            // if less than threshold, print warning (only on rt threads)
            if (end > time && ec_cfg.protocol == "iddp"){
                ++overruns;
                DPRINTF(
                    "OVR #%d wake=%lld read=%lld trj + set_ref=%lld "
                    "SM=%lld write=%lld log=%lld total=%lld late=%lld us\n",
                    overruns,
                    static_cast<long long>(to_us(t0 - scheduled_release)),
                    static_cast<long long>(to_us(t1 - t0)),
                    static_cast<long long>(to_us(t2 - t1)),
                    static_cast<long long>(to_us(t3 - t2)),
                    static_cast<long long>(to_us(t4 - t3)),
                    static_cast<long long>(to_us(end - t4)),
                    static_cast<long long>(to_us(end - t0)),
                    static_cast<long long>(to_us(end - time))
                );
            }
#endif
            std::this_thread::sleep_until(time);
        }           
    }
    
    ec_wrapper.stop_ec_sys();

    if(fatal_error!=""){
        DPRINTF("%s\n",fatal_error.c_str());
        return 1;
    }
    
    return 0;
}
