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
   _subscriber->setsockopt(ZMQ_RCVHWM, &opt_hwm, sizeof ( opt_hwm ) );
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
    try{
        bool read_message=true;
        int more=1;
        size_t more_size = sizeof(more);
        std::string msg_id="";
        while(read_message){
            zmq::message_t message;
            if(!_subscriber->recv(&message,ZMQ_DONTWAIT)){
                read_message=false;
            }
            else{
                _subscriber->getsockopt(ZMQ_RCVMORE, &more, &more_size);
                if (!more){
                    if(msg_id!=""){
                        pb_rx_pdos.Clear();
                        pb_rx_pdos.ParseFromArray(message.data(),message.size());
                        get_from_pb();
                        read_message=false; //  Last message frame
                    }
                }else{
                    msg_id= std::string(static_cast<char*> (message.data()), message.size());
                }
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
