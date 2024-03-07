#include <cassert>
#include <tuple>

#include "udpSock.h"
#include "protocols/udp/ec_udp.h"
#include "pck_msgs.h"

#include <magic_enum.hpp>

using boost::asio::ip::udp;
using namespace std::chrono_literals;
using namespace std::chrono;


/**
 * @brief EcUDP::EcUDP
 */
EcUDP::EcUDP(std::string host_address,uint32_t host_port) :
       EcBoostCmd("EcUDP",host_address,host_port),
       EcBoostPdo("EcUDP",host_address,host_port)
{
    _actual_server_status= ServerStatus::IDLE;
}

EcUDP::~EcUDP()
{
    stop_client();
    
    if(_ec_udp_thread != nullptr)
    {
        if ( _ec_udp_thread->joinable() ) 
        {
            _consoleLog->info("EtherCAT Client thread stopped");
            _ec_udp_thread->join();
        }
    }
    
    _consoleLog->info("That's all folks");
    _consoleLog.reset();
}


//******************************* EVENT HANDLERS *****************************************************//

void EcUDP::set_loop_time(uint32_t period_ms)
{
    auto period_ms_time=milliseconds(period_ms);
    
    EcBoostPdo::set_period(period_ms_time);
}

void EcUDP::start_client(uint32_t period_ms,bool logging)
{

    _consoleLog->info(" EtherCAT Client UDP Started " + make_daytime_string());
    
    set_loop_time(period_ms);
        
    connect();
    
    _logging=logging;
    SSI slave_info;
    if(retrieve_slaves_info(slave_info)){
        try{
            esc_factory(slave_info);
            if(_logging){
                _ec_logger->init_mat_logger(slave_info);
                start_logging();
            }
        } catch ( std::exception &e ) {
            DPRINTF ( "Fatal Error: %s\n", e.what() );
            stop_client();
        }
    }

    if(_ec_udp_thread == nullptr)
    {
        _ec_udp_thread = std::make_shared<std::thread>(std::thread{[&]{EcBoostPdo::run();}});
    }
    else
    {
        // leave the periodicActivity alive with a period changed
    }
}

void EcUDP::stop_client()
{
    if(_client_alive)
    {
        stop_logging();
        
        EcBoostPdo::stop();
        
        disconnect();
        
        _client_alive =false;
    }
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
//     auto sample_time = steady_clock::now();
//     
//     // Server alive checking //
//     auto client_alive_elapsed_ms=duration_cast<milliseconds>(sample_time-_client_alive_time);
//     
//     if(_actual_server_status==ServerStatus::IDLE)
//     {
//         if(client_alive_elapsed_ms >= _server_alive_check_ms)
//         {
//              // Stop to receive motors, imu, ft, power board pdo information // 
//             // reset motors references
//             _motor_ref_flags = RefFlags::FLAG_NONE;
//             //_motors_references.clear();
//             
//             // stop client
//             stop_client();
//         }
//     }
//     else
//     {
//         _actual_server_status= ServerStatus::IDLE;
//         _client_alive_time = steady_clock::now();
//         
//         // Receive motors, imu, ft, power board pdo information // 
// 
//        send_pdo();
//     }
        
}
//******************************* Periodic Activity *****************************************************//

void EcUDP::receive_error(std::error_code ec)
{
    _consoleLog->error( " Receive Error {}", ec.message());
}



