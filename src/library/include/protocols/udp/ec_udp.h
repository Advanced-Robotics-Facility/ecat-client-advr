#ifndef EC_UDP_H
#define EC_UDP_H


#include "common/boost/ec_boost_cmd.h"
#include "common/boost/ec_boost_pdo.h"
/////////////////////////////////////////////////////

/**
 * @brief The ReplServer class
 */
class EcUDP : public EcBoostCmd, public EcBoostPdo
{
public:

    EcUDP(std::string host_address,uint32_t host_port);
    ~EcUDP();

    void start_client(uint32_t period_ms,bool logging) final;
    void stop_client() final ;
    void set_loop_time(uint32_t period_ms) final;
    
    void receive_error(std::error_code ec);
    void periodicActivity();

private:
    std::chrono::steady_clock::time_point _client_alive_time; 
    std::shared_ptr<std::thread> _ec_udp_thread;
    std::vector<MR> _mot_ref_tv; //vector of tuple
};

#endif // EC_UDP_H
