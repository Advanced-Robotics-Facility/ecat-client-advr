#include "mechanism/zmq/ec_zmq_pdo.h"
#include <iostream>

using namespace zmq;
using namespace iit::advr;
using namespace std;

std::unique_ptr<zmq::context_t> EcZmqPdoContext::pdo_context;

EcZmqPdo::EcZmqPdo( int32_t id, uint32_t type, const std::string zmq_uri):
_id(id),
_type(type),
_zmq_uri(zmq_uri)
{   
}
    
EcZmqPdo::EcZmqPdo( int32_t id, const std::string esc_name, const std::string zmq_uri):
_id(id),
_esc_name(esc_name),
_zmq_uri(zmq_uri)
{   
}

EcZmqPdo::EcZmqPdo(const std::string zmq_uri):
_zmq_uri(zmq_uri)
{   
}

std::string EcZmqPdo::get_zmq_pdo_uri()
{
    return _zmq_uri;
}

void EcZmqPdo::init(void)
{
    try{
        _subscriber = std::make_shared<socket_t>(*EcZmqPdoContext::pdo_context, ZMQ_SUB);
        _subscriber->setsockopt(ZMQ_SUBSCRIBE, "",0); 
    }catch ( zmq::error_t& e ) { 
        std::string zmq_error(e.what());
        throw std::runtime_error("error on subscriber socket initialization: "+zmq_error);
    }
}

int EcZmqPdo::write_connect(void)
{
    int ret=0;
    try{
        printf("...connecting to zmq uri: [%s]\n",_zmq_uri.c_str());
        _subscriber->connect(_zmq_uri);
    }catch ( zmq::error_t& e ) { 
        std::cout << "fatel error on subscriber socket connection: " << e.what() << std::endl;
        ret=-1;
    }
    return ret;
}

int EcZmqPdo::write_quit(void)
{
    int ret=0;
    try{
        printf("...disconnecting zmq uri: [%s]\n",_zmq_uri.c_str());
        _subscriber->disconnect(_zmq_uri);
    }catch ( zmq::error_t& e ) { 
        std::cout << "fatel error on subscriber socket disconnection: " << e.what() << std::endl;
        ret=-1;
    }
    return 0;
}

int EcZmqPdo::read()
{
    try{
        if(_subscriber->recv(&_zmq_msg_id,ZMQ_RCVMORE)){
            if(_subscriber->recv(&_zmq_msg)){
                pb_rx_pdos.ParseFromArray(_zmq_msg.data(),_zmq_msg.size());
                get_from_pb();
                return _zmq_msg.size();
            }
        }
    }catch(std::exception& e){
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}

int EcZmqPdo::write()
{
    set_to_pb();
    // publish to zmq socket
    return 0;
}
