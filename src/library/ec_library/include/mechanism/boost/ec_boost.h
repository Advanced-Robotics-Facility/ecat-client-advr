#ifndef EC_BOOST
#define EC_BOOST

#include <boost/asio.hpp>
#include "udpSock.h"
#include "task.h"
#include "pck_msgs.h"
#include <magic_enum.hpp>
using boost::asio::ip::udp;

class EcBoost : public UdpTask<EcBoost, MsgPackProtocol>
{
public:

    EcBoost(std::string task_name,std::string host_address,uint32_t host_port):
        UdpTask(task_name, host_port-1){
        
        _client_port= host_port-1;    
        
        if(host_address=="localhost"){
            host_address.clear();
            host_address="127.0.0.1";
        }
        sender_endpoint.address(boost::asio::ip::address::from_string(host_address));
        sender_endpoint.port(host_port);
        
        // Register Message Handler
        registerHandler(UdpPackMsg::MSG_SRV_REP,    &EcBoost::server_replies_handler);
        registerHandler(UdpPackMsg::MSG_SRV_STS,    &EcBoost::server_status_handler);
        registerHandler(UdpPackMsg::MSG_REPL_REP,   &EcBoost::repl_replies_handler);
        registerHandler(UdpPackMsg::MSG_MOTOR_STS,  &EcBoost::motor_status_handler);
        registerHandler(UdpPackMsg::MSG_FT6_STS,    &EcBoost::ft6_status_handler);
        registerHandler(UdpPackMsg::MSG_PWR_STS,    &EcBoost::pwr_status_handler);
    }
    
    virtual ~EcBoost(){
    };
    
    virtual void receive_error(std::error_code ec)=0;
    virtual void periodicActivity()=0;
    
protected:
    virtual void server_replies_handler(char*buf, size_t size)=0;
    virtual void repl_replies_handler(char*buf, size_t size)=0;
    virtual void server_status_handler(char *buf, size_t size)=0;
    virtual void motor_status_handler(char*buf, size_t size)=0;
    virtual void ft6_status_handler(char*buf, size_t size)=0;
    virtual void pwr_status_handler(char*buf, size_t size)=0;
    
    uint32_t _client_port;

};

#endif
