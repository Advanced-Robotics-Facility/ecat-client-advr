#include "protocol/udp/ec_udp.h"
using boost::asio::ip::udp;
using namespace std::chrono_literals;
using namespace std::chrono;

/**
 * @brief EcUDP::EcUDP
 */
EcUDP::EcUDP(std::string host_address,uint32_t host_port) :
       EcBoost("EcUDP",host_address,host_port)
{
    _actual_server_status= ServerStatus::IDLE;

    _client_thread_info.policy=SCHED_OTHER;
    _client_thread_info.priority=sched_get_priority_max ( _client_thread_info.policy ) / 2;
}

EcUDP::~EcUDP()
{
    stop_client();
    
    if(_ec_udp_thread != nullptr){
        if ( _ec_udp_thread->joinable() ) {
            _consoleLog->info("EtherCAT Client thread stopped");
            _ec_udp_thread->join();
        }
    }
}


//******************************* EVENT HANDLERS *****************************************************//

void EcUDP::set_loop_time(uint32_t period_ms)
{
    auto period_ms_time=milliseconds(period_ms);
    
    set_period(period_ms_time);
}

void EcUDP::start_client(uint32_t period_ms)
{

    _consoleLog->info(" EtherCAT Client UDP Started " + make_daytime_string());
    
    set_loop_time(period_ms);
        
    connect();
    _client_alive_time = steady_clock::now();
    if(_ec_udp_thread == nullptr){
        _client_status.run_loop=true;
        _ec_udp_thread = std::make_shared<std::thread>(std::thread{[&]{run();}});
        
        sched_param sch;
        sch.sched_priority = _client_thread_info.priority;
        if (pthread_setschedparam(_ec_udp_thread->native_handle(), _client_thread_info.policy, &sch)){
            std::string error_code(std::strerror(errno));
            throw std::runtime_error("Failed to setschedparam: " + error_code);
        }
        
        std::stringstream udp_thread_id;
        udp_thread_id << _ec_udp_thread->get_id();
        _client_thread_info.cpu=sched_getcpu();
        DPRINTF("Client thread initialized, ");
        DPRINTF("id: %s cpu: %d, priority %d\n",udp_thread_id.str().c_str(),_client_thread_info.cpu,_client_thread_info.priority);
  
    }
    else{
        // leave the periodicActivity alive with a period changed
    }

    SSI slave_info;
    if(retrieve_slaves_info(slave_info)){
        try{
            esc_factory(slave_info);
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }
}

void EcUDP::stop_client()
{
    if(_client_status.run_loop){
        stop();
    }
        
    disconnect();
    
    _client_status.run_loop=false;
    _client_status.status=ClientStatusEnum::NOT_ALIVE;
}



//******************************* Periodic Activity *****************************************************//
#ifdef PROFILE_TIMING
auto lastTime_ = std::chrono::high_resolution_clock::now();
#endif
/**
 * @brief Client::periodicActivity
 */
void EcUDP::periodicActivity()
{
    auto sample_time = steady_clock::now();
    
    // Server alive checking //
    auto client_alive_elapsed_ms=duration_cast<milliseconds>(sample_time-_client_alive_time);

    if(_actual_server_status==ServerStatus::IDLE){
        if(client_alive_elapsed_ms >= _server_alive_check_ms){
            // Stop receiving motors, imu, ft, power board pdo information // 
            // Stop sending motors references
            // Stop client
            stop_client();
        }
    }
    else{
        _actual_server_status= ServerStatus::IDLE;
        _client_alive_time = steady_clock::now();
        
        // read motors, imu, ft, power board and others pdo information 
    }
        
}
//******************************* Periodic Activity *****************************************************//

void EcUDP::write()
{
    // send motors and others pdo
    send_pdo();
}

void EcUDP::receive_error(std::error_code ec)
{
    _consoleLog->error( " Receive Error {}", ec.message());
}



