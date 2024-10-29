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
 
    try{
        ec_wrapper.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
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

        std::map<int, double> motors_set_zero,motors_set_ref, motors_start;
        std::map<int, double> valves_set_zero, valves_set_ref, valves_start;
        std::map<int,Trj_ptr> general_trj;

        int trajectory_counter=0;
        float tau=0,alpha=0;

        for (const auto &[esc_id, valve_rx_pdo] : valve_status_map){
            if(ec_cfg.trj_config_map.count("valve")>0){
                if(ec_cfg.trj_config_map["valve"].set_point.count(esc_id)>0){
                    std::string set_point_type="";
                    if(ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_POSITION){
                        set_point_type="position";
                        valves_start[esc_id] = std::get<0>(valve_rx_pdo); // actual encoder position
                    }else if(ec_cfg.device_config_map[esc_id].control_mode_type==iit::advr::Gains_Type_IMPEDANCE){
                        set_point_type="force";
                        valves_set_zero[esc_id]=valves_start[esc_id] = 0.0;
                    }else{
                        set_point_type="current";
                        valves_set_zero[esc_id]=valves_start[esc_id] = 0.0;
                    }
                    
                    if(ec_cfg.trj_config_map["valve"].set_point[esc_id].count(set_point_type)>0){
                        general_trj[esc_id]= ec_cfg.trj_config_map["valve"].trj_generator[esc_id][set_point_type];
                        valves_set_ref[esc_id]= valves_start[esc_id];
                    }
                }
            }
        }

        for (const auto &[esc_id, motor_rx_pdo] : motor_status_map){
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

                    if(ec_cfg.trj_config_map["motor"].trj_generator[esc_id].count(set_point_type)>0){
                        general_trj[esc_id]= ec_cfg.trj_config_map["motor"].trj_generator[esc_id][set_point_type];
                        motors_set_ref[esc_id]= motors_start[esc_id];
                    }
                }
            }
        }

        if(motors_set_ref.empty() && valves_set_ref.empty()){
            throw std::runtime_error("fatal error: motor references and valves references are both empty");
        }
        // memory allocation
                
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
            throw std::runtime_error(e.what());
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
                        target=valves_set_zero[esc_id];
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
                        target=motors_set_zero[esc_id];
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
    
    return 0;
}
