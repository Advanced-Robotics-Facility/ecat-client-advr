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

    std::map<int,Trj_ptr> general_trj;
    for (auto &[esc_id, motor_trj] : motor_trj_map){
        general_trj[esc_id] = motor_trj.general_trj;
    }
    for (auto &[esc_id, valve_trj] : valve_trj_map){
        general_trj[esc_id] = valve_trj.general_trj;
    }

    if(general_trj.empty()){
        DPRINTF("fatal error: general trajectory map empty\n");
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
    
        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);

        for (auto &[esc_id, current_trj] : general_trj){ 
            current_trj->reset();
            current_trj->set_start_time();
        }
        
        while (run_loop && client->get_client_status().run_loop){
            client->read();

            // Valves references
            for (auto &[esc_id, current_trj] : general_trj){
                double target=0;
                if (!current_trj->ended()) {
                    target= current_trj->operator()();
                }else{
                    run_loop=false;
                }
                
                int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                if(valve_reference_map.count(esc_id)>0){
                    if(!run_loop){
                        target=valve_trj_map[esc_id].set_zero;
                        if(target == iit::advr::Gains_Type_POSITION){
                            client->get_valve_status(valve_status_map);
                            target=std::get<0>(valve_status_map[esc_id]); // actual encoder position
                        }
                    }
                    if(ctrl_mode == iit::advr::Gains_Type_POSITION){
                    std::get<1>(valve_reference_map[esc_id]) = target;
                    }else if(ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                        std::get<2>(valve_reference_map[esc_id]) = target;
                    }else{
                        std::get<0>(valve_reference_map[esc_id]) = target;
                    }
                }
                else if(motor_reference_map.count(esc_id)>0){
                    if(!run_loop){
                        target=motor_trj_map[esc_id].set_zero;
                        if(ctrl_mode == iit::advr::Gains_Type_POSITION ||
                            ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                            client->get_motor_status(motor_status_map);
                            target=std::get<2>(motor_status_map[esc_id]); // actual motor pos
                        }
                    }
                    if(ctrl_mode != iit::advr::Gains_Type_VELOCITY){
                        if(ctrl_mode == iit::advr::Gains_Type_POSITION ||
                            ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                            std::get<1>(motor_reference_map[esc_id]) = target;
                        }
                        if(ctrl_mode != iit::advr::Gains_Type_POSITION &&
                            ctrl_mode != iit::advr::Gains_Type_IMPEDANCE){
                            std::get<3>(motor_reference_map[esc_id]) = target; // current mode (0xCC or oxDD) or impedance
                        }
                    }else{
                        std::get<2>(motor_reference_map[esc_id]) = target;
                    }
                }
            }

            // ************************* SEND ALWAYS REFERENCES***********************************//
            if(!valve_reference_map.empty()){
                client->set_valve_reference(valve_reference_map);
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//
                 
            // ************************* SEND ALWAYS REFERENCES***********************************//
            if(!motor_reference_map.empty()){
                client->set_motor_reference(motor_reference_map);
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//

            time = time + period;


            client->write();
            ec_wrapper.log_ec_sys();
            
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

    if(fatal_error!=""){
        DPRINTF("%s\n",fatal_error.c_str());
        return 1;
    }
    
    return 0;
}
