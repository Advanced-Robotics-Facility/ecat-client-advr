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

#include "rt_thread.h"
#include "nrt_thread.h"

using slave_descr_t = std::vector<std::tuple<int, int, int>>;

#define SIG_TEST 

static int loop_guard = 1;

static const std::string rt2nrt_pipe( "RT2NRT" );
static const std::string nrt2rt_pipe( "NRT2RT" );


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
    int num_wrk;
    int rt_th_period_us;
    int timeout_s;
    std::string hardware_type;
    
    try
    {
        options.add_options()
            ("v, verbose", "verbose", cxxopts::value<bool>()->implicit_value("true"))
            ("p, rt_th_period_us", "rt_th_period_us", cxxopts::value<int>()->default_value("1000"))
            ("t, timeout_s", "timeout_s", cxxopts::value<int>()->default_value("1"))
            ("h, hardware_type", "hardware_type", cxxopts::value<std::string>()->default_value("idle"));
        auto result = options.parse(argc, argv);

        rt_th_period_us = result["rt_th_period_us"].as<int>();
        timeout_s = result["timeout_s"].as<int>();
        hardware_type = result["hardware_type"].as<std::string>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }
    
    if((hardware_type != "idle") && (hardware_type != "pos") && (hardware_type != "vel") && (hardware_type != "imp"))
    {
        throw std::runtime_error("hardware type not recognized");
    }
        

#ifdef SIG_TEST
    sigset_t set;
    siginfo_t info;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    struct timespec ts = { timeout_s, 0 };
    main_common (&argc, (char*const**)&argv, 0);
#else
    main_common (&argc, (char*const**)&argv, test_sighandler);
#endif
    
    std::string rt_thread_start=hardware_type;
    std::string RT_name = "RT_thread";
    threads[RT_name] = new RT_motor_thread(RT_name,
                                           rt_thread_start,
                                           nrt2rt_pipe, rt2nrt_pipe,
                                           rt_th_period_us);
        
    /*
    * The barrier is opened when COUNT waiters arrived.
    */
    //pthread_barrier_init(&threads_barrier, NULL, threads.size()+1 );    

    threads[RT_name]->create(true);

    std::cout << "... wait on barrier of EC_SIM TEST" << std::endl;
    //sleep(3);
    //pthread_barrier_wait(&threads_barrier);
    
    //int wait_ret;
        
    #ifdef SIG_TEST
        #ifdef __COBALT__
            // here I want to catch CTRL-C 
            //wait_ret=__real_sigtimedwait(&set,&info,&ts);
            __real_sigwait(&set, &sig);
        #else
            //timespec
            //wait_ret=sigtimedwait(&set,&info,&ts);
            sigwait(&set, &sig);  
        #endif
    #else
        while ( loop_guard ) {
            sleep(1);
        }
    #endif
    
    threads["RT_thread"]->stop();
    
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
