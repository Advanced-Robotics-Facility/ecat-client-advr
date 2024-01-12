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

#include <test_common.h>

#include "ec_rt_trajectory.h"

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
    
    int timeout_s;
    std::string hardware_type;
    
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
    
    EcUtils::Ptr ec_client_utils;
    EcUtils::EC_CONFIG ec_client_cfg;

    try{
        ec_client_utils=std::make_shared<EcUtils>();
        ec_client_cfg = ec_client_utils->get_ec_cfg();
    }catch(std::exception &ex){
        std::cout << "Error on ec client config file" << std::endl;
        std::cout << ex.what() << std::endl;
    return 1;
    }
    

    // *************** START CLIENT  *************** //
    EcIface::Ptr client=ec_client_utils->make_ec_iface();
    
    threads["EcRtTrajectory"] = new EcRtTrajectory(ec_client_cfg.period_ms,
                                                    ec_client_cfg,
                                                    client);
        
    threads["EcRtTrajectory"]->create(true);

        
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
    
    threads["EcRtTrajectory"]->stop();
    
    for ( auto const &t : threads) {
        //t.second->stop();
        t.second->join();
        delete t.second;
    }
    
    // STOP CLIENT
    if(client->is_client_alive())
    {
        client->stop_client();
    }
    
    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;
}
