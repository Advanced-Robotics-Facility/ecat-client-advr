#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/sysinfo.h>

#include <exception>
#include <iostream>
#include <cmath>
#include <random>

#include "utils/ec_common_step.h"
#include <cxxopts.hpp>
#include <test_common.h>

#include "rt_esc_pipe.h"
#include "nrt_esc_zmq.h"

#define SIG_TEST 


static int loop_guard = 1;


EcThreadsMap threads;
    
static void test_sighandler(int signum) {
    
    std::cout << "Handling signal " << signum << std::endl;
    loop_guard = 0;
}




////////////////////////////////////////////////////
//
// Main
//
////////////////////////////////////////////////////

int main ( int argc, char * argv[] ) try {

    EcUtils::EC_CONFIG ec_cfg;
    EcCommonStep ec_common_step;
    
    try{
        ec_cfg=ec_common_step.retrieve_ec_cfg();
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }

#ifdef SIG_TEST
    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    main_common (&argc, (char*const**)&argv, 0);
#else
    main_common (&argc, (char*const**)&argv, test_sighandler);
#endif
    
    uint32_t rt_th_period_us=ec_cfg.period_ms*1000;
    
    std::map<int,int> esc_map;
    for ( auto &[esc_id, esc_type, pos] : ec_cfg.fake_slave_info ) {
        esc_map[esc_id]=esc_type;
    }
    
    std::string thread_name="";
    if(ec_cfg.protocol=="iddp"){
        thread_name="RtEscPipe";
        threads[thread_name] = new RtEscPipe("NoNe", esc_map,rt_th_period_us);
        threads[thread_name]->create(true);
    }
    else{
        thread_name="NrtEscZmq";
        threads[thread_name] = new NrtEscZmq(esc_map,rt_th_period_us,ec_cfg.protocol,ec_cfg.host_name,ec_cfg.host_port+4000);
        threads[thread_name]->create(false);
    }

#ifdef SIG_TEST
    #ifdef __COBALT__
        // here I want to catch CTRL-C 
        __real_sigwait(&set, &sig);
    #else
        sigwait(&set, &sig);  
    #endif
#else
    while ( loop_guard ) {
        sleep(1);
    }
#endif

    threads[thread_name]->stop();
    
    for ( auto const &t : threads) {
        //t.second->stop();
        t.second->join();
        delete t.second;
    }
    
    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}
