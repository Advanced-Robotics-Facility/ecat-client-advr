#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>
#include <semaphore.h>

#include "utils/ec_wrapper.h"
#include <test_common.h>

using namespace std::chrono;

EcIface::Ptr client;
EcUtils::EC_CONFIG ec_cfg;

static bool run_loop = true;
static pthread_mutex_t reference_mutex;
static sem_t writer_ready;

// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    run_loop=false;
}

//void on_motor_command(const MotorCommandMsg& msg)
void on_motor_command()
{
    if(ec_cfg.protocol=="iddp"){
        pthread_mutex_lock(&reference_mutex);
    }

    //motor_reference_map = convert_motors(msg);
    //gripper_reference_map = convert_grippers(msg);

    if(ec_cfg.protocol=="iddp"){
        pthread_mutex_unlock(&reference_mutex);
        sem_post(&writer_ready);
    }
}

void set_reference(){
    if(!motor_reference_map.empty()){
        client->set_motor_reference(motor_reference_map);
    }

    if(!gripper_reference_map.empty()){
        client->set_gripper_reference(gripper_reference_map);
    }
}

class EcWriter : public EcThread
{
public:

    EcWriter(){

        pthread_mutexattr_t reference_mutex_attr;

        pthread_mutexattr_init(&reference_mutex_attr);
        pthread_mutexattr_settype(&reference_mutex_attr, PTHREAD_MUTEX_NORMAL);
        pthread_mutexattr_setpshared(&reference_mutex_attr, PTHREAD_PROCESS_PRIVATE);
        pthread_mutexattr_setprotocol(&reference_mutex_attr, PTHREAD_PRIO_NONE);
        
        int ret=0;
        bool error=false;
        ret=pthread_mutex_init(&reference_mutex, &reference_mutex_attr);
        if (ret != 0){
            pthread_mutexattr_destroy(&reference_mutex_attr);
            throw std::runtime_error("fatal error: cannot initialize reference_mutex, reason: "+std::to_string(ret));
        }

        sem_init(&writer_ready, 0, 0);

        auto client_thread_info = client->get_client_thread_info();

        schedpolicy = client_thread_info.policy;
        priority = std::max(0,client_thread_info.priority-1);
        // non-periodic
        period.period = {0,1}; 
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    }

    void stop_writer(){

        _run_loop = 0;
        sem_post(&writer_ready);

        join();
        pthread_mutex_destroy(&reference_mutex);
        sem_destroy(&writer_ready);
    }

    ~EcWriter() {
    }
public:
    
    virtual void th_init ( void * ){
        DPRINTF("Writer thread initialized, ");
        DPRINTF("id: %ld cpu: %d, priority %d\n",pthread_self(),sched_getcpu(),priority);   
    }   

    virtual void th_loop ( void * ){
        while (run_loop && client->get_client_status().run_loop){
            if (sem_wait(&writer_ready) == 0) {
                if (!run_loop)
                    break;

                DPRINTF("Write cmd\n");
                pthread_mutex_lock(&reference_mutex);
                set_reference();
                pthread_mutex_unlock(&reference_mutex);
                
                client->write();
            }
        }

        _run_loop = 0;
    }
};

int main(int argc, char * const argv[])
{
    EcWrapper ec_wrapper;
    std::string fatal_error="";
    try{
        ec_wrapper.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
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

    EcWriter ec_writer;

    if(ec_sys_started){                       

        if(motor_trj_map.empty() && gripper_trj_map.empty()){
            fatal_error="fatal error: devices references structure empty!";
            run_loop=false;
        }else{     
            if (ec_cfg.protocol == "iddp"){
                auto writer_cpu = client->get_client_thread_info().cpu;
                ec_writer.create(true,writer_cpu);
                std::this_thread::sleep_for(100ms);
            }

            struct sigaction sa;
            sa.sa_handler = sig_handler;
            sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
            sigaction(SIGINT,&sa, nullptr);
        }

        using Clock = std::chrono::steady_clock;

        auto start_time = Clock::now();
        auto time = start_time;
        const auto period = std::chrono::nanoseconds(4* 1000000); // 4ms default

        while (run_loop && client->get_client_status().run_loop){

            client->read();
            ec_wrapper.read_devices_status();
            
            // ros2 publisher
            //.......................
            //.......................
            //.......................
            // ros2 subscriber
            on_motor_command();

            if (ec_cfg.protocol != "iddp"){
                set_reference();
                client->write();
            }

            ec_wrapper.log_ec_sys();

            time = time + period;
            std::this_thread::sleep_until(time);
        }           
    }

    ec_writer.stop_writer();
    ec_wrapper.stop_ec_sys();

    if(fatal_error!=""){
        DPRINTF("%s\n",fatal_error.c_str());
        return 1;
    }
    
    return 0;
}
