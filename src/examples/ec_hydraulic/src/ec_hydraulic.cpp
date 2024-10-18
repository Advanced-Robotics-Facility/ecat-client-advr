#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_wrapper.h"
#include <test_common.h>
#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02

using namespace std::chrono;

int main(int argc, char *const argv[])
{
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcWrapper ec_wrapper;

    try{
        ec_wrapper.create_ec(client, ec_cfg);
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }

    std::map<int,double> homing,trajectory;
    if(ec_cfg.trj_config_map.count("valve")>0){
        homing=ec_cfg.trj_config_map["motor"].homing;    
        trajectory=ec_cfg.trj_config_map["motor"].trajectory;    
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
        float pressure_time_ms = 1000; // 2minutes.
        float set_trj_time_ms = hm_time_ms;

        std::string STM_sts;
        bool run_loop = true;

        std::map<int, double> motors_trj_1,motors_trj_2, motors_set_zero, motors_set_trj;
        std::map<int, double> motors_set_ref, motors_start;

        std::map<int, uint8_t> pumps_trj_1, pumps_set_trj;
        std::map<int, uint8_t> pumps_set_ref, pumps_start, pumps_actual_read;
        uint16_t pump_status_word;
        int pump_req_op = PUMP_PRE_OP;
        double error_pressure_set;
        bool pump_req_sts, pump_in_pressure;

        std::map<int, double> valves_trj_1, valves_trj_2, valves_set_zero, valves_set_trj;
        std::map<int, double> valves_set_ref, valves_start;

        int trajectory_counter = 0;
        float tau = 0, alpha = 0;

        // memory allocation
        for (const auto &[esc_id, pump_rx_pdo] : pump_status_map){
            if(ec_cfg.trj_config_map.count("pump")>0){
                if(ec_cfg.trj_config_map["pump"].set_point.count(esc_id)>0){
                    pumps_trj_1[esc_id] = static_cast<uint8_t>(ec_cfg.trj_config_map["pump"].set_point[esc_id]["pressure"]);
                    pumps_set_ref[esc_id] = pumps_start[esc_id] = pumps_actual_read[esc_id] = std::get<0>(pump_rx_pdo);
                    pumps_set_trj[esc_id] = pumps_trj_1[esc_id];
                }
            }
        }
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
                        valves_trj_1[esc_id] =ec_cfg.trj_config_map["valve"].set_point[esc_id][set_point_type];
                        valves_trj_2[esc_id] = -1*valves_trj_1[esc_id];
                        valves_set_trj[esc_id]=   valves_trj_1[esc_id];
                        valves_set_ref[esc_id]= valves_start[esc_id];
                    }
                }
            }
        }

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

        if (motors_set_ref.empty() && valves_set_ref.empty() && pumps_set_ref.empty()){
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
                client->set_pumps_references(pumps_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Valves references
            if (!valves_ref.empty()){
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
                    // interpolate
                    for (const auto &[esc_id, target] : valves_set_trj){
                        int ctrl_mode= ec_cfg.device_config_map[esc_id].control_mode_type;
                        valves_set_ref[esc_id] = valves_start[esc_id] + alpha * (target - valves_start[esc_id]);

                        if(ctrl_mode == iit::advr::Gains_Type_POSITION){
                            std::get<1>(valves_ref[esc_id]) = valves_set_ref[esc_id];
                        }else if(ctrl_mode == iit::advr::Gains_Type_IMPEDANCE){
                            std::get<2>(valves_ref[esc_id]) = valves_set_ref[esc_id];
                        }else{
                            std::get<0>(valves_ref[esc_id]) = valves_set_ref[esc_id];
                        }
                    }
                }
                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_valves_references(valves_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            // Motors references
            if (!motors_ref.empty())
            {
                if (STM_sts == "Homing" || STM_sts == "Trajectory"){
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
                }
                // ************************* SEND ALWAYS REFERENCES***********************************//
                client->set_motors_references(motors_ref);
                // ************************* SEND ALWAYS REFERENCES***********************************//
            }

            time = time + period;

            if (STM_sts == "PumpPreOp"){
                if (pump_req_sts){
                    if (trajectory_counter == ec_cfg.repeat_trj){
                        run_loop = false;
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
                        run_loop = false;
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
                        run_loop = false;
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
                    run_loop = false;
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
                            pumps_start =   pumps_set_ref;

                            tau = alpha = 0;
                            trajectory_counter = ec_cfg.repeat_trj; // exit
                        }
                        else{
                            STM_sts = "Homing";
                            start_time = time;
                            set_trj_time_ms = hm_time_ms;

                            motors_set_trj = motors_trj_1;
                            motors_start =   motors_set_ref;

                            valves_set_trj = valves_trj_1;
                            valves_start =   valves_set_ref;

                            tau = alpha = 0;
                        }
                    }
                }
            }
            else if ((time_elapsed_ms >= hm_time_ms) && (STM_sts == "Homing")){
                STM_sts = "Trajectory";
                start_time = time;
                set_trj_time_ms = trj_time_ms;

                motors_start =   motors_set_ref;
                motors_set_trj = motors_trj_2;

                valves_start =   valves_set_ref;
                valves_set_trj = valves_trj_2;


                if (trajectory_counter == ec_cfg.repeat_trj - 1){
                    if(!motors_set_zero.empty()){
                        motors_set_trj = motors_set_zero;
                    }
                    if(!valves_set_zero.empty()){
                        valves_set_trj = valves_set_zero; 
                    }
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
                        pumps_start =   pumps_set_ref;

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

                    motors_set_trj = motors_trj_1;
                    motors_start =   motors_set_ref;

                    valves_set_trj = valves_trj_1;
                    valves_start =   valves_set_ref;

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
