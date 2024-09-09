#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_common_step.h"
#include <test_common.h>
#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02

using namespace std::chrono;

int main(int argc, char *const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcCommonStep ec_common_step;

    try{
        ec_common_step.create_ec(client, ec_cfg);
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }

    if (!ec_cfg.homing_position.empty()){
        if (ec_cfg.trajectory.empty()){
            DPRINTF("Please setup a trajectory map!\n");
            return 1;
        }
    }

    bool ec_sys_started = true;
    try{
        ec_sys_started = ec_common_step.start_ec_sys();
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }

    if (ec_sys_started){
        int overruns = 0;
        float time_elapsed_ms;
        float hm_time_ms = ec_cfg.homing_time_sec * 1000;
        float trj_time_ms = ec_cfg.trajectory_time_sec * 1000;
        float pressure_time_ms = 1000; // 2minutes.
        float set_trj_time_ms = hm_time_ms;

        std::string STM_sts;
        bool run = true;

        std::map<int, double> q_set_trj = ec_cfg.homing_position;
        std::map<int, double> q_ref, q_start, qdot;
        double qdot_ref_k = 1.0; // [rad/s]
        std::map<int, double> qdot_ref, qdot_start,qdot_set_trj;
        std::map<int, double> qdot_set_trj_1,qdot_set_trj_2,qdot_set_zero;

        double taur_ref_k = 1.0; // [Nm]
        std::map<int, double> tor_ref, tor_start, tor_set_trj;
        std::map<int, double> tor_set_trj_1,tor_set_trj_2,tor_set_zero;

        uint8_t pump_pressure_ref = 180; // bar
        std::map<int, uint8_t> pumps_trj_1, pumps_set_trj;
        std::map<int, uint8_t> pumps_set_ref, pumps_start, pumps_actual_read;
        uint16_t pump_status_word;
        int pump_req_op = PUMP_PRE_OP;
        double error_pressure_set;
        bool pump_req_sts, pump_in_pressure;

        double valve_curr_ref = 2.5; // mA
        std::map<int, double> valves_trj_1, valves_trj_2, valves_set_zero, valves_set_trj;
        std::map<int, double> valves_set_ref, valves_start;

        int trajectory_counter = 0;
        float tau = 0, alpha = 0;

        // memory allocation
        for (const auto &[esc_id, pump_rx_pdo] : pump_status_map){
            for (auto pump_id : ec_cfg.pump_id){
                if(pump_id==esc_id){
                    pumps_trj_1[esc_id] = pump_pressure_ref;
                    pumps_set_ref[esc_id] = pumps_start[esc_id] = pumps_actual_read[esc_id] = std::get<0>(pump_rx_pdo);
                    break;
                }
            }
        }

        for (const auto &[esc_id, valve_rx_pdo] : valve_status_map){
            for (auto valve_id : ec_cfg.valve_id){
                if(valve_id==esc_id){
                    valves_trj_1[esc_id] = valve_curr_ref;
                    valves_trj_2[esc_id] = -valve_curr_ref;
                    valves_set_zero[esc_id] = 0.0;
                    valves_set_ref[esc_id] = valves_start[esc_id] = valves_set_zero[esc_id];
                    break;
                }
            }
        }

        for (const auto &[esc_id, motor_rx_pdo] : motors_status_map){
            if(ec_cfg.homing_position.count(esc_id)>0){
                q_start[esc_id] = std::get<1>(motors_status_map[esc_id]); // motor pos
                qdot[esc_id] = std::get<3>(motors_status_map[esc_id]);    // motor vel
                q_ref[esc_id] = q_start[esc_id];

                qdot_ref[esc_id] = qdot_start[esc_id] = 0.0;
                qdot_set_trj_1[esc_id] = qdot_ref_k;
                qdot_set_trj_2[esc_id] = -qdot_ref_k;
                qdot_set_trj[esc_id] = qdot_set_trj_1[esc_id];
                qdot_set_zero[esc_id]=0.0;

                tor_ref[esc_id] = tor_start[esc_id] = 0.0;
                tor_set_trj_1[esc_id] = taur_ref_k;
                tor_set_trj_2[esc_id] = -taur_ref_k;
                tor_set_trj[esc_id] = tor_set_trj_1[esc_id];
                tor_set_zero[esc_id]=0.0;
            }
        }

        pumps_set_trj = pumps_trj_1;
        // pumps references check
        for (const auto &[esc_id, press_ref] : pumps_set_ref){
            std::get<0>(pumps_ref[esc_id])=press_ref;
            std::get<8>(pumps_ref[esc_id])=pump_req_op;
        }

        valves_set_trj = valves_trj_1;
        // valves references check
        for (const auto &[esc_id, curr_ref] : valves_set_ref){
            std::get<0>(valves_ref[esc_id])=curr_ref;
        }
        // motors references check
        if (!q_ref.empty()){
            if (motors_ref.empty()){
                throw std::runtime_error("fatal error: motors references structure empty!");
            }
        }

        if (q_ref.empty() && valves_set_ref.empty() && pumps_set_ref.empty()){
            throw std::runtime_error("fatal error: motor references, pump reference and valves references are both empty");
        }

        if (!pumps_ref.empty()){
            STM_sts = "PumpPreOp";
        }
        else{
            STM_sts = "Homing";
            set_trj_time_ms = hm_time_ms;
        }
        // memory allocation

        if (ec_cfg.protocol == "iddp"){
            DPRINTF("Real-time process....\n");
            main_common(&argc, (char *const **)&argv, 0);
            int priority = SCHED_OTHER;
            #if defined(PREEMPT_RT) || defined(__COBALT__)
                priority = sched_get_priority_max ( SCHED_FIFO ) / 3;
            #endif
            int ret = set_main_sched_policy(priority);
            if (ret < 0){
                throw std::runtime_error("fatal error on set_main_sched_policy");
            }
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);

        while (run && client->is_client_alive()) {
            client->read();
            ec_common_step.telemetry();

            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Main Time elapsed ms: [%f]\n",time_elapsed_ms);

            client->get_pump_status(pump_status_map);
            pump_req_sts = true;
            for (const auto &[esc_id, pump_rx_pdo] : pump_status_map){
                pumps_actual_read[esc_id] = std::get<0>(pump_rx_pdo);
                pump_status_word = std::get<1>(pump_rx_pdo);
                if (pump_status_word != pump_req_op){
                    pump_req_sts &= false;
                }

#ifdef TEST_EXAMPLES
                if (ec_cfg.protocol != "iddp"){
                    pump_req_sts = true;
                }
#endif
            }

            // define a simplistic linear trajectory
            tau = time_elapsed_ms / set_trj_time_ms;
            // quintic poly 6t^5 - 15t^4 + 10t^3
            alpha = ((6 * tau - 15) * tau + 10) * tau * tau * tau;

            // Pump references
            if (!pumps_ref.empty()){
                if (STM_sts == "Pressure"){
                    // interpolate
                    for (const auto &[esc_id, target] : pumps_set_trj){
                        pumps_set_ref[esc_id] = pumps_start[esc_id] + alpha * (target - pumps_start[esc_id]);
                        std::get<0>(pumps_ref[esc_id]) = pumps_set_ref[esc_id];
                        std::get<8>(pumps_ref[esc_id]) = pump_req_op;
                    }
                }

                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_pumps_references(RefFlags::FLAG_MULTI_REF, pumps_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Valves references
            if (!valves_ref.empty()){
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
                    // interpolate
                    for (const auto &[esc_id, target] : valves_set_trj){
                        valves_set_ref[esc_id] = valves_start[esc_id] + alpha * (target - valves_start[esc_id]);
                        std::get<0>(valves_ref[esc_id]) = valves_set_ref[esc_id];
                    }
                }

                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Motors references
            if (!motors_ref.empty())
            {
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
                    // interpolate
                    for (const auto &[esc_id, target] : q_set_trj){
                        int ctrl_mode= std::get<0>(motors_ref[esc_id]);
                        if(ctrl_mode != iit::advr::Gains_Type_VELOCITY){
                            if(ctrl_mode == iit::advr::Gains_Type_POSITION ||
                               ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                                q_ref[esc_id] = q_start[esc_id] + alpha * (q_set_trj[esc_id] - q_start[esc_id]);
                                std::get<1>(motors_ref[esc_id]) = q_ref[esc_id];
                            }
                            if(ctrl_mode != iit::advr::Gains_Type_POSITION ){
                                tor_ref[esc_id] = tor_start[esc_id] + alpha * (tor_set_trj[esc_id] - tor_start[esc_id]);
                                std::get<3>(motors_ref[esc_id]) = tor_ref[esc_id]; // current mode (0xCC or oxDD) or impedance
                            }
                        }
                        else{
                            qdot_ref[esc_id] = qdot_start[esc_id] + alpha * (qdot_set_trj[esc_id] - qdot_start[esc_id]);
                            std::get<2>(motors_ref[esc_id]) = qdot_ref[esc_id];
                        }
                    }
                }

                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            time = time + period;

            if (STM_sts == "PumpPreOp"){
                if (pump_req_sts){
                    if (trajectory_counter == ec_cfg.repeat_trj){
                        run = false;
                    }
                    else{
                        STM_sts = "PumpOp";
                        pump_req_op = PUMP_OP;
                        start_time = time;
                        tau = alpha = 0;
                    }
                }
                else{
                    if (time_elapsed_ms >= 500){ // 500ms
                        DPRINTF("Cannot setup the pump in pre-operational mode\n");
                        run = false;
                    }
                }
            }
            else if (STM_sts == "PumpOp"){
                if (pump_req_sts){
                    STM_sts = "Pressure";
                    start_time = time;
                    set_trj_time_ms = pressure_time_ms;
                    tau = alpha = 0;
                }
                else{
                    if (time_elapsed_ms >= 500){ // 500ms
                        DPRINTF("Cannot setup the pump in operational mode\n");
                        run = false;
                    }
                }
            }
            else if ((time_elapsed_ms >= pressure_time_ms) && (STM_sts == "Pressure")){
                pump_in_pressure = true;
                for (const auto &[esc_id, press_ref] : pumps_set_ref){
                    error_pressure_set = std::abs(press_ref - pumps_actual_read[esc_id]);
                    if (error_pressure_set >= 2.0){ // 2bar
                        pump_in_pressure &= false;
                        DPRINTF("Pump id: %d has an error on demanded pressure= %f\n", esc_id, error_pressure_set);
                        break;
                    }
                }
#ifdef TEST_EXAMPLES
                if (ec_cfg.protocol != "iddp"){
                    pump_in_pressure = true;
                }
#endif
                if (!pump_in_pressure){
                    run = false;
                }
                else{
                    if (trajectory_counter == ec_cfg.repeat_trj){
                        STM_sts = "PumpPreOp";
                        pump_req_op = PUMP_PRE_OP;
                        start_time = time;
                        tau = alpha = 0;
                    }
                    else{
                        if (motors_ref.empty() && valves_ref.empty()){
                            STM_sts = "Pressure";
                            start_time = time;
                            set_trj_time_ms = pressure_time_ms;

                            pumps_set_trj = pumps_start;
                            pumps_start = pumps_set_ref;

                            tau = alpha = 0;
                            trajectory_counter = ec_cfg.repeat_trj; // exit
                        }
                        else{
                            STM_sts = "Homing";
                            start_time = time;
                            set_trj_time_ms = hm_time_ms;

                            q_set_trj = ec_cfg.homing_position;
                            q_start = q_ref;

                            qdot_set_trj = qdot_set_trj_1;
                            qdot_start = qdot_ref;

                            tor_set_trj = tor_set_trj_1;
                            tor_start = tor_ref;

                            valves_set_trj = valves_trj_1;
                            valves_start = valves_set_ref;

                            tau = alpha = 0;
                        }
                    }
                }
            }
            else if ((time_elapsed_ms >= hm_time_ms) && (STM_sts == "Homing")){
                STM_sts = "Trajectory";
                start_time = time;
                set_trj_time_ms = trj_time_ms;

                q_set_trj = ec_cfg.trajectory;
                q_start = q_ref;

                
                qdot_start = qdot_ref;
                tor_start = tor_ref;
                valves_start = valves_set_ref;

                if (trajectory_counter == ec_cfg.repeat_trj - 1){ // second last references
                    valves_set_trj = valves_set_zero;             // set to zero. (close valves)
                    qdot_set_trj = qdot_set_zero;
                    tor_set_trj = tor_set_zero;
                }
                else{
                    qdot_set_trj = qdot_set_trj_2;
                    tor_set_trj = tor_set_trj_2;
                    valves_set_trj = valves_trj_2;
                }

                tau = alpha = 0;
                trajectory_counter = trajectory_counter + 1;
            }
            else if ((time_elapsed_ms >= trj_time_ms) && (STM_sts == "Trajectory"))
            {
                if (trajectory_counter == ec_cfg.repeat_trj){
                    if (!pumps_ref.empty()){
                        STM_sts = "Pressure";
                        start_time = time;
                        set_trj_time_ms = pressure_time_ms;

                        pumps_set_trj = pumps_start;
                        pumps_start = pumps_set_ref;

                        tau = alpha = 0;
                    }
                    else{
                        run = false; // only homing or trajectory on valves/motors
                    }
                }
                else{
                    STM_sts = "Homing";
                    start_time = time;
                    set_trj_time_ms = hm_time_ms;            

                    q_set_trj = ec_cfg.homing_position;
                    q_start = q_ref;

                    qdot_set_trj = qdot_set_trj_1;
                    qdot_start = qdot_ref;

                    tor_set_trj = tor_set_trj_1;
                    tor_start = tor_ref;

                    valves_set_trj = valves_trj_1;
                    valves_start = valves_set_ref;

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

    ec_common_step.stop_ec_sys();

    return 0;
}
