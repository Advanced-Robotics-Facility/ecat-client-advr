#include "mechanism/zmq/ec_zmq_pdo.h"
#include <iostream>

using namespace zmq;
using namespace iit::advr;
using namespace std;

namespace EcZmqContext{
    static zmq::context_t sub_context(1);
}

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
    try{
        _subscriber = std::make_shared<socket_t>(EcZmqContext::sub_context, ZMQ_SUB);
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
        zmq::message_t zmq_msg_id;
        while(_subscriber->recv(&zmq_msg_id,ZMQ_RCVMORE)){
            //_msg_id= std::string(static_cast<char*> (zmq_msg_id.data()), zmq_msg_id.size());
            zmq::message_t zmq_msg;
            if(_subscriber->recv(&zmq_msg)){
                pb_rx_pdos.Clear();
                pb_rx_pdos.ParseFromArray(zmq_msg.data(),zmq_msg.size());
                get_from_pb();
            }
        }
    }catch(std::exception& e){
        std::cout << e.what() << std::endl;
        return 0;
    }
    return 0;
}

int EcZmqPdo::write()
{
    set_to_pb();
    // publish to zmq socket
    return 0;
}
