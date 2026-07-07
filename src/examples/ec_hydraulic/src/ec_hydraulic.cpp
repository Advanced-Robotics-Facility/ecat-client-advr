#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_wrapper.h"
#include <test_common.h>
#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02

using namespace std::chrono;

static bool run_loop = true;
// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    run_loop=false;
}

int main(int argc, char *const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcWrapper ec_wrapper;
    std::string fatal_error="";
    try{
        ec_wrapper.create_ec(client, ec_cfg);
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
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

    if (ec_sys_started){
        int overruns = 0;
        float time_elapsed_ms;
        float hm_time_ms = ec_cfg.trj_time * 1000;
        float trj_time_ms = hm_time_ms;
        float pressure_time_ms = 60000; // 2minutes.
        float set_trj_time_ms = hm_time_ms;

        std::string STM_sts;

        double error_pressure_set;
        bool pump_in_pressure;

        int trajectory_counter = 0;
        float tau = 0, alpha = 0;

        if (motor_trj_map.empty() && valve_trj_map.empty() && pump_trj_map.empty()){
            fatal_error="fatal error: motor references, pump reference and valves references are both empty";
            run_loop=false;
        }else{
            if (!pump_reference_map.empty()){
                STM_sts = "Pressure";
                set_trj_time_ms = pressure_time_ms;
            }
            else{
                STM_sts = "Homing";
                set_trj_time_ms = hm_time_ms;
            }
            
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

        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);

        while (run_loop && client->get_client_status().run_loop){
            client->read();
            
            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Time elapsed ms: [%f]\n",time_elapsed_ms);

            client->get_pump_status(pump_status_map);

            // define a simplistic linear trajectory
            tau = time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6 * tau - 15) * tau + 10) * tau * tau * tau;
            alpha = std::clamp(alpha, 0.0f, 1.0f);

            // Pump references
            if (!pump_reference_map.empty()){
                if (STM_sts == "Pressure"){
                    // interpolate
                    for (auto &[esc_id, pump_trj] : pump_trj_map){
                        pump_trj.set_ref = pump_trj.start + alpha * (pump_trj.set_trj - pump_trj.start);
                        std::get<0>(pump_reference_map[esc_id]) = pump_trj.set_ref;
                        std::get<6>(pump_reference_map[esc_id]) = 1;
                    }
                }
                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_pump_reference(pump_reference_map);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Valves references
            if (!valve_reference_map.empty()){
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
                    // interpolate
                    for (auto &[esc_id, valve_trj] : valve_trj_map){
                        int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                        valve_trj.set_ref = valve_trj.start + alpha * (valve_trj.set_trj - valve_trj.start);

                        if(ctrl_mode == iit::advr::Gains_Type_POSITION){
                            std::get<1>(valve_reference_map[esc_id]) = valve_trj.set_ref;
                        }else if(ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                            std::get<2>(valve_reference_map[esc_id]) = valve_trj.set_ref;
                            std::get<0>(valve_reference_map[esc_id]) = -1.0; //current fdw ref
                        }else{
                            std::get<0>(valve_reference_map[esc_id]) = valve_trj.set_ref;
                        }
                    }
                }
                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_valve_reference(valve_reference_map);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Motors references
            if (!motor_reference_map.empty())
            {
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
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
                }
                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_motor_reference(motor_reference_map);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            time = time + period;

            if ((time_elapsed_ms >= pressure_time_ms) && (STM_sts == "Pressure")){
                pump_in_pressure = true;
                for (auto &[esc_id, pump_trj] : pump_trj_map){
                    error_pressure_set = std::abs(pump_trj.set_ref - std::get<2>(pump_status_map.at(esc_id))); //pressure1;
                    if (error_pressure_set >= 5.0){ // 5bar
                        pump_in_pressure &= false;
                        DPRINTF("Pump id: %d has an error on demanded pressure= %f\n", esc_id, error_pressure_set);
                        break;
                    }
                }
                if (!pump_in_pressure || trajectory_counter == ec_cfg.repeat_trj){
                    run_loop = false;
                }
                else{
                    if (motor_reference_map.empty() && valve_reference_map.empty()){
                        STM_sts = "Pressure";
                        start_time = time;
                        set_trj_time_ms = pressure_time_ms;

                        set_esc_trj(pump_trj_map,TrjType::start);
                    
                        tau = alpha = 0;
                        trajectory_counter = ec_cfg.repeat_trj; // exit
                    }
                    else{
                        STM_sts = "Homing";
                        start_time = time;
                        set_trj_time_ms = hm_time_ms;

                        set_esc_trj(motor_trj_map,TrjType::trj1);
                        set_esc_trj(valve_trj_map,TrjType::trj1);

                        tau = alpha = 0;
                    }
                }
            }
            else if ((time_elapsed_ms >= hm_time_ms) && (STM_sts == "Homing")){
                STM_sts = "Trajectory";
                start_time = time;
                set_trj_time_ms = trj_time_ms;

                if (trajectory_counter == ec_cfg.repeat_trj - 1){
                    set_esc_trj(motor_trj_map,TrjType::zero);
                    set_esc_trj(valve_trj_map,TrjType::zero);

                }else{
                    set_esc_trj(motor_trj_map,TrjType::trj2);
                    set_esc_trj(valve_trj_map,TrjType::trj2);
                }

                tau = alpha = 0;
                trajectory_counter = trajectory_counter + 1;
            }
            else if ((time_elapsed_ms >= trj_time_ms) && (STM_sts == "Trajectory"))
            {
                if (trajectory_counter == ec_cfg.repeat_trj){
                    if (!pump_reference_map.empty()){
                        STM_sts = "Pressure";
                        start_time = time;
                        set_trj_time_ms = pressure_time_ms;

                        set_esc_trj(pump_trj_map,TrjType::start);

                        tau = alpha = 0;
                    }
                    else{
                        run_loop= false; // only homing or trajectory on valves/motors
                    }
                }
                else{
                    STM_sts = "Homing";
                    start_time = time;
                    set_trj_time_ms = hm_time_ms;            

                    set_esc_trj(motor_trj_map,TrjType::trj1);
                    set_esc_trj(valve_trj_map,TrjType::trj1);

                    tau = alpha = 0;
                }
            }

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
