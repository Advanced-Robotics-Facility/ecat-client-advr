#include "mechanism/zmq/ec_zmq_pdo.h"
#include <iostream>

using namespace zmq;
using namespace iit::advr;
using namespace std;


EcZmqPdo::EcZmqPdo( int32_t id, uint32_t type, const std::string zmq_uri):
_id(id),
_type(type),
_zmq_uri(zmq_uri)
{   
    printf("PDO ZMQ URI: [%s]\n",zmq_uri.c_str());
}
    
EcZmqPdo::EcZmqPdo( int32_t id, const std::string esc_name, const std::string zmq_uri):
_id(id),
_esc_name(esc_name),
_zmq_uri(zmq_uri)
{   
    printf("PDO ZMQ URI: [%s]\n",zmq_uri.c_str());
}

EcZmqPdo::EcZmqPdo(const std::string zmq_uri):
_zmq_uri(zmq_uri)
{   
    printf("PDO ZMQ URI: [%s]\n",zmq_uri.c_str());
}

std::string EcZmqPdo::get_zmq_pdo_uri()
{
    return _zmq_uri;
}

void EcZmqPdo::init(void)
{
    #if ZMQ_VERSION_MAJOR == 2
    uint64_t opt_hwm = 1;
    #else
    int opt_hwm = 1;
    #endif

    _context = std::make_shared<context_t>(1);
    _subscriber = std::make_shared<socket_t>(*_context, ZMQ_SUB);

    std::string key("");

   _subscriber->setsockopt(ZMQ_SUBSCRIBE, key.c_str(),key.length()); 
   //_subscriber->setsockopt(ZMQ_RCVHWM, &opt_hwm, sizeof ( opt_hwm ) );
   //_subscriber->setsockopt(ZMQ_RCVBUF, 2*1024);
}

int EcZmqPdo::write_connect(void)
{
    _subscriber->connect(_zmq_uri);
    return 0;
}

int EcZmqPdo::write_quit(void)
{
    return 0;
}

int EcZmqPdo::read()
{
    zmq::multipart_t multipart;
    std::string msg="";
    //int nmsg = 0;
    try{
        while(multipart.recv(*_subscriber,ZMQ_DONTWAIT))
        {
            //nmsg++;
            msg="";
            if(!multipart.empty())
            {
                msg = multipart.popstr();
                auto zmqmsg=multipart.pop();
                auto msg_size=zmqmsg.size();
                pb_rx_pdos.Clear();
                pb_rx_pdos.ParseFromArray(zmqmsg.data(),msg_size);
            } 
        }
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return 0;
    }
    
    if(msg==""){
        pb_rx_pdos.Clear();
    }
    else{
        get_from_pb();
    }

    //if(nmsg>100)
    //    std::cout << "recv " << nmsg << " from zmq" << std::endl;

    return 0;
}

int EcZmqPdo::write()
{
    set_to_pb();
    // publish to zmq socket
    return 0;
}
