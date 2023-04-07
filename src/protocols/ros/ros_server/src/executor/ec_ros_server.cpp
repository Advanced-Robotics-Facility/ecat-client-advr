#include <iostream>
#include <csignal>
#include <atomic>
#include <boost/asio.hpp>
#include <stdlib.h>
#include "ec_client_ros.h"


using namespace std;

atomic_bool run(true);

/* signal handler*/
void on_sigint(int)
{
    run = false;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ec_client",ros::init_options::NoSigintHandler);
    
    // catch ctrl+c
    struct sigaction sa;
    sa.sa_handler = on_sigint;
    sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
    sigaction(SIGINT,&sa, nullptr);

    cout << "ec_client_main started..." << endl;
    
    /*** ec_client ros object initialization *///
    string zmq_uri="tcp://"+boost::asio::ip::host_name()+":5555";
    int timeout=1000; // 1s of timeout;
    
    EC_Client_ROS::Ptr ec_client_ros;

    ec_client_ros.reset();
    ec_client_ros = std::make_shared<EC_Client_ROS>(zmq_uri,timeout);
    
    /*** ec_client ros object initialization *///

    /*** ROS LOOP of 1KHz *///
    ros::Rate loop_rate(1000);
    
    // wait for ctrl+c
    while(run)
    {
        /* STATE MACHINE of XBotCore presence to limit the ROS services */
        ec_client_ros->xbotcore_presence_STM();
        
        /* ZMQ communication Error! Restart the communication */
        if(!ec_client_ros->callAvailable())
        {
            cout << "Problem with the communication, I'm going to restart it!" << endl;
            ec_client_ros->start_zmq_mechanism();            

        }
        /* Publish the ec ec_client information (Robot and PDOs) */
        ec_client_ros->pub_ec_client_info();
        
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    cout << "ec_client_main stopped..." << endl;
}
