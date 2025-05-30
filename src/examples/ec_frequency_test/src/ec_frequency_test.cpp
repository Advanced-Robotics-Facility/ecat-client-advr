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
    std::string fatal_error="";
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

    unsigned long int increment_us = 0;
    if(argv[1] != NULL){
        char* p;
        increment_us = strtoul(argv[1], &p, 10);
        if (*p != '\0' || errno != 0) {
            DPRINTF("Problem on frequency parameter!\n");
            return 1;
        }
    }

    if (increment_us > ULONG_MAX) {
        DPRINTF("Frequency parameter is not valid!\n");
        return 1;
    }else{
        DPRINTF("Frequency parameter is: %ld us\n",increment_us);
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
                motors_set_ref[esc_id] = std::get<2>(motor_rx_pdo); // motor pos];
            }
        }

        if(motors_set_ref.empty()){
            fatal_error="fatal error: motors references structure empty!";
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
       
        auto start_time = std::chrono::high_resolution_clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);
        auto incrementat_freq = std::chrono::nanoseconds(0);
        auto incrementat_k=std::chrono::nanoseconds(increment_us*1000);
        
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
                incrementat_freq = incrementat_freq + incrementat_k;
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

    if(fatal_error!=""){
        DPRINTF("%s\n",fatal_error.c_str());
        return 1;
    }
    
    return 0;
}
