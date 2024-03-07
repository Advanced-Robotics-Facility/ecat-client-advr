#ifndef EC_BOOST_PDO_H
#define EC_BOOST_PDO_H

#include <boost/asio.hpp>

#include "ec_iface.h"
#include "task.h"
#include "pck_msgs.h"
using boost::asio::ip::udp;

class EcBoostPdo : public UdpTask<EcBoostPdo, MsgPackProtocol>,virtual public EcIface
{
public:

    EcBoostPdo(std::string task_name,std::string host_address,uint32_t host_port);
    ~EcBoostPdo();
    
    virtual void receive_error(std::error_code ec)=0;
    virtual void periodicActivity()=0;
    void esc_factory(SSI slave_descr);
    
protected:
    ServerStatus _actual_server_status;
private:  
    // MSG HANDLERS
    void server_replies_handler(char*buf, size_t size);
    void server_status_handler(char *buf, size_t size);
    void motor_status_handler(char*buf, size_t size);
    void ft6_status_handler(char*buf, size_t size);
    void pwr_status_handler(char*buf, size_t size);
};

#endif
