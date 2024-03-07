#ifndef EC_BOOST
#define EC_BOOST

#include <boost/asio.hpp>
#include "udpSock.h"
#include "ec_iface.h"
#include "task.h"
#include "pck_msgs.h"
using boost::asio::ip::udp;

class EcBoost : virtual public UdpTask<EcBoost, MsgPackProtocol>,virtual public EcIface
{
public:

    EcBoost(std::string task_name,std::string host_address,uint32_t host_port):
    UdpTask(task_name, host_port-1){
        if(host_address=="localhost"){
            host_address.clear();
            host_address="127.0.0.1";
        }
        sender_endpoint.address(boost::asio::ip::address::from_string(host_address));
        sender_endpoint.port(host_port);
    }
    ~EcBoost(){
    };
};

#endif
