#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include <atomic>

#include "utils/ec_wrapper.h"
#include <test_common.h>

using namespace std::chrono;

static bool run_loop = true;
// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    switch (sig)
    {
    case SIGALRM:
        run_loop = false;
        break;
    
    default:
        run_loop = false;
        break;
    }
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

    std::map<int,double> homing;
    if(ec_cfg.trj_config_map.count("motor")>0){
        homing=ec_cfg.trj_config_map["motor"].homing;    
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
        float time_elapsed_ms;
        
        bool run=true;        
        std::map<int, double> motors_set_ref;

        for (const auto &[esc_id, motor_rx_pdo] : motor_status_map){
            if(homing.count(esc_id)){
                motors_set_ref[esc_id] = std::get<1>(motor_status_map[esc_id]); // motor pos];
            }
        }

        if(motors_set_ref.empty()){
            throw std::runtime_error("fatal error: motors references structure empty!");
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
        auto incrementat_freq = std::chrono::nanoseconds(0);
        auto incrementat_k=std::chrono::nanoseconds(100000); //(100 us) every 1s
        bool incrementat_freq_req=true;
        
        while (run_loop && client->get_client_status().run_loop){
            client->read();

            time_elapsed_ms = std::chrono::duration<float, std::milli>(time - start_time).count();
            //DPRINTF("Time elapsed ms: [%f]\n",time_elapsed_ms);
            

            // ************************* SEND ALWAYS REFERENCES***********************************//
            for ( const auto &[esc_id, pos_ref] : motors_set_ref){
                std::get<1>(motor_reference_map[esc_id]) = pos_ref;
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//
            client->set_motor_reference(motor_reference_map);
            // ************************* SEND ALWAYS REFERENCES***********************************//

            time = time + period + incrementat_freq;

            if(time_elapsed_ms>=1000){ //1s
                if(incrementat_freq_req){
                    incrementat_freq = incrementat_freq + incrementat_k;
                }
                start_time=time;
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
    
    return 0;
}
