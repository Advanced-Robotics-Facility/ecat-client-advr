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

#define SIG_TEST 


static int loop_guard = 1;


ThreadsMap threads;
    
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
    // MOTOR
    for(int i=0;i<ec_cfg.motor_id.size();i++){
        int motor_id=ec_cfg.motor_id[i];
        esc_map[motor_id]=iit::ecat::CENT_AC;
    }
    
    // IMU
    for(int i=0;i<ec_cfg.imu_id.size();i++){
        int imu_id=ec_cfg.imu_id[i];
        esc_map[imu_id]=iit::ecat::IMU_ANY;
    }
    
    // FT
    for(int i=0;i<ec_cfg.ft_id.size();i++){
        int ft_id=ec_cfg.ft_id[i];
        esc_map[ft_id]=iit::ecat::FT6_MSP432;
    }
    
    // POW
    for(int i=0;i<ec_cfg.pow_id.size();i++){
        int pow_id=ec_cfg.pow_id[i];
        esc_map[pow_id]=iit::ecat::POW_F28M36_BOARD;
    }
    
    threads["RtEscPipe"] = new RtEscPipe("NoNe", esc_map,rt_th_period_us);

    threads["RtEscPipe"]->create(true);

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

    threads["RtEscPipe"]->stop();
    
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
