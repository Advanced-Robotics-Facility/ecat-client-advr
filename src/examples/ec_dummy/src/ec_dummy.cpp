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

#include <cxxopts.hpp>
#include <test_common.h>

#include "rt_motor.h"

/*
 * Use protobuf msg iit::advr::Repl_cmd
 * - set type ad iit::advr::CmdType::CTRL_CMD
 * - set header
 * 
 */
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

    cxxopts::Options options(argv[0], " - wizardry setup");
    int motor_numb;
    int rt_th_period_us;
    
    try
    {
        options.add_options()
        ("m, motor_numb", "motor_numb", cxxopts::value<int>()->default_value("5"))
        ("p, rt_th_period_us", "rt_th_period_us", cxxopts::value<int>()->default_value("1000"));
        auto result = options.parse(argc, argv);

        motor_numb = result["motor_numb"].as<int>();
        rt_th_period_us = result["rt_th_period_us"].as<int>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
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
    
    std::map<int,int> motor_map;
    for(int i=1;i<motor_numb+1;i++){
        motor_map[i]=iit::ecat::CENT_AC;
    }
    
    threads["RtMotor"] = new RtMotor("NoNe", motor_map,rt_th_period_us);

    threads["RtMotor"]->create();

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

    threads["RtMotor"]->stop();
    
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
