#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "ec_client_utils.h"

#include "client.h"


// EtherCAT Motor type
#define LO_PWR_DC_MC 0x12
#define CENT_AC 0x15

std::atomic_bool run(true);

void on_sigint(int)
{
    run = false;
}


using namespace std::chrono;

int main()
{
     EC_Client_Utils::Ptr ec_client_utils;
     EC_Client_Utils::EC_CONFIG ec_client_cfg;

    // Find the ec client configuration environment variable for setting.
    char* ec_client_cfg_path;
    ec_client_cfg_path = getenv ("EC_CLIENT_CFG");

    if (ec_client_cfg_path==NULL)
    {
        std::cout << "EC Client configuration is not found"
                     ", please setup it using the environment variable with name: EC_CLIENT_CFG " << std::endl;
    }
    else
    {
        try{
            
        auto ec_client_cfg_file = YAML::LoadFile(ec_client_cfg_path);
        ec_client_utils=std::make_shared<EC_Client_Utils>(ec_client_cfg_file);
        ec_client_cfg = ec_client_utils->get_ec_client_config();  
            
        }catch(std::exception &ex){
        std::cout << "Error on ec client config file" << std::endl;
        std::cout << ex.what() << std::endl;
        return 1;
        }
        createLogger("console","client");

        Client client(ec_client_cfg.host_name_s,ec_client_cfg.host_port);

        auto UDP_period_ms_time=milliseconds(ec_client_cfg.UDP_period_ms);

        client.connect();

        client.set_period(UDP_period_ms_time);

        // Run asio thread alongside main thread
        std::thread t1{[&]{client.run();}};
        
        // catch ctrl+c
        struct sigaction sa;
        sa.sa_handler = on_sigint;
        sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
        sigaction(SIGINT,&sa, nullptr);

        
        auto start_time= system_clock::now();
        //auto time=start_time;
        // wait for ctrl+c
            
        while (run)
        {
            auto time= system_clock::now();
            auto time_elapsed_ms= duration_cast<milliseconds>(time-start_time);
            
            client.ping(true);   
            // delay until time to iterate again
            time += UDP_period_ms_time;
            
            std::cout << time_elapsed_ms.count() << std::endl;
            
            std::this_thread::sleep_until(time);
        }
        
        spdlog::get("console")->info("That's all folks");
        client.stop_client();
        
        if ( t1.joinable() ) 
        {
            std::cout << "Client thread stopped" << std::endl;
            t1.join();
        }
    }

    return 0;
}
