#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_wrapper.h"
#include <test_common.h>

using namespace std::chrono;

int main(int argc, char * const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcWrapper ec_wrapper;
 
    try{
        ec_wrapper.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }

    std::map<int,double> homing,trajectory;
    if(ec_cfg.trj_config_map.count("valve")>0){
        homing=ec_cfg.trj_config_map["motor"].homing;    
        trajectory=ec_cfg.trj_config_map["motor"].trajectory;    
    }

    if(homing.empty()){
        DPRINTF("Got an homing position map\n");
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
        
        bool run_loop=true;        
        std::string STM_sts="Homing";

        std::map<int, double> motors_trj_1,motors_trj_2, motors_set_zero, motors_set_trj;
        std::map<int, double> motors_set_ref, motors_start;

        int trajectory_counter=0;
        float tau=0,alpha=0;

        for (const auto &[esc_id, motor_rx_pdo] : motors_status_map){
            if(ec_cfg.trj_config_map.count("motor")>0){
                if(ec_cfg.trj_config_map["motor"].set_point.count(esc_id)>0){
                    std::string set_point_type="";
                    if(ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_POSITION ||
                       ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_IMPEDANCE){
                        set_point_type="position";
                        motors_start[esc_id]=std::get<1>(motor_rx_pdo); // actual motor pos
                    }else if(ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_VELOCITY){
                        set_point_type="velocity";
                        motors_set_zero[esc_id]=motors_start[esc_id]=0.0;
                    }else if(ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_TORQUE){
                        set_point_type="torque";
                        motors_set_zero[esc_id]=motors_start[esc_id]=0.0;
                    }else{
                        set_point_type="current";
                        motors_set_zero[esc_id]=motors_start[esc_id]=0.0;
                    }

                    if(ec_cfg.trj_config_map["motor"].set_point[esc_id].count(set_point_type)>0){
                        if(set_point_type=="position"){
                            motors_trj_1[esc_id]=homing[esc_id];
                            motors_trj_2[esc_id]=trajectory[esc_id];    
                            motors_set_zero[esc_id]=motors_trj_1[esc_id];             
                        }else{
                            motors_trj_1[esc_id]=ec_cfg.trj_config_map["motor"].set_point[esc_id][set_point_type];
                            motors_trj_2[esc_id]=-1*motors_trj_1[esc_id];
                        }
                        motors_set_trj[esc_id]= motors_trj_1[esc_id];
                        motors_set_ref[esc_id]= motors_start[esc_id];
                    }
                }
            }
        }

        if(motors_set_ref.empty()){
            throw std::runtime_error("fatal error: motors references structure empty!");
        }
        // memory allocation
                
        if (ec_cfg.protocol == "iddp"){
            //avoid map swap
            main_common(&argc, (char *const **)&argv, 0);
        }

        // process scheduling
        try{
            ec_wrapper.ec_self_sched(argv[0]);
        }catch(std::exception& e){
            throw std::runtime_error(e.what());
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);
        
        while (run_loop && client->get_client_status().run_loop){
            client->read();
            
            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Time elapsed ms: [%f]\n",time_elapsed_ms);
        
            // define a simplistic linear trajectory
            tau= time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;
           // interpolate
            for (const auto &[esc_id, target] : motors_set_trj){
                int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                motors_set_ref[esc_id] = motors_start[esc_id] + alpha * (target - motors_start[esc_id]);
                if(ctrl_mode != iit::advr::Gains_Type_VELOCITY){
                    if(ctrl_mode == iit::advr::Gains_Type_POSITION ||
                        ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                        std::get<1>(motors_ref[esc_id]) = motors_set_ref[esc_id];
                    }
                    if(ctrl_mode != iit::advr::Gains_Type_POSITION &&
                        ctrl_mode != iit::advr::Gains_Type_IMPEDANCE){
                        std::get<3>(motors_ref[esc_id]) = motors_set_ref[esc_id]; // current mode (0xCC or oxDD) or impedance
                    }
                }else{
                    std::get<2>(motors_ref[esc_id]) = motors_set_ref[esc_id];
                }
            }            
            // ************************* SEND ALWAYS REFERENCES***********************************//
            client->set_motors_references(motors_ref);
            // ************************* SEND ALWAYS REFERENCES***********************************//

            time = time + period;

            if((time_elapsed_ms>=hm_time_ms)&&(STM_sts=="Homing")) {
                STM_sts = "Trajectory";
                start_time = time;
                set_trj_time_ms = trj_time_ms;

                motors_start =   motors_set_ref;
                motors_set_trj = motors_trj_2;

                if (trajectory_counter == ec_cfg.repeat_trj - 1){
                    if(!motors_set_zero.empty()){
                        motors_set_trj = motors_set_zero;
                    }
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
                    
                    motors_set_trj = motors_trj_1;
                    motors_start =   motors_set_ref;

                    tau = alpha = 0;
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
    
    ec_wrapper.stop_ec_sys();
    
    return 0;
}
