#include "mechanism/zmq/ec_zmq_pdo.h"
#include <esc_info.h>
#include <iostream>

using namespace zmq;
using namespace iit::advrf;
using namespace std;

std::unique_ptr<zmq::context_t> EcZmqPdoContext::pdo_context;

EcZmqClient::EcZmqClient() = default;

void EcZmqClient::init_subscriber(const std::string& uri)
{
    if (!EcZmqPdoContext::pdo_context) {
        throw std::logic_error("ZMQ context is not initialized");
    }

    try {
        auto subscriber =std::make_unique<zmq::socket_t>(*EcZmqPdoContext::pdo_context,
                                                          zmq::socket_type::sub);

        subscriber->set(zmq::sockopt::linger,0);

        subscriber->set(zmq::sockopt::rcvtimeo,1000);

        subscriber->set(zmq::sockopt::subscribe,"");

        printf("...connecting to subscriber zmq uri: [%s]\n",uri.c_str());
        subscriber->connect(uri);

        _subscriber = std::move(subscriber);

    } catch (const zmq::error_t& error) {
        throw std::runtime_error(
            "Error initializing subscriber socket at '" +
            uri + "': " + error.what()
        );
    }
}

void EcZmqClient::init_dealer(const std::string& uri)
{

    if (!EcZmqPdoContext::pdo_context) {
        throw std::logic_error("ZMQ context is not initialized");
    }

    try {
        auto dealer = std::make_unique<zmq::socket_t>(*EcZmqPdoContext::pdo_context,
                                                       zmq::socket_type::dealer);

        dealer->set(zmq::sockopt::linger,0);

        dealer->set(zmq::sockopt::sndtimeo,100);

        dealer->set(zmq::sockopt::rcvtimeo,100);

        printf("...connecting to dealer zmq uri: [%s]\n",uri.c_str());
        dealer->connect(uri);

        _dealer = std::move(dealer);

    } catch (const zmq::error_t& error) {
        throw std::runtime_error(
            "Error initializing dealer socket at '" +
            uri + "': " + error.what()
        );
    }
}

void EcZmqClient::initialize(std::string subscriber_uri,std::string dealer_uri){

    _subscriber_uri = subscriber_uri;
    _dealer_uri = dealer_uri;

    init_subscriber(_subscriber_uri);
    //init_dealer(_dealer_uri);
}

void EcZmqClient::stop(){

    if (_subscriber) {
        try{
            printf("...disconnecting subscriber zmq uri: [%s]\n",_subscriber_uri.c_str());
            _subscriber->disconnect(_subscriber_uri);
        }catch ( zmq::error_t& e ) { 
            std::cout << "fatel error on subscriber socket disconnection: " << e.what() << std::endl;
        }
    }

    if (_dealer) {
        try{
            printf("...disconnecting dealer zmq uri: [%s]\n",_dealer_uri.c_str());
            _dealer->disconnect(_dealer_uri);
        }catch ( zmq::error_t& e ) { 
            std::cout << "fatel error on dealer socket disconnection: " << e.what() << std::endl;
        }
    }

    _subscriber.reset();
    _dealer.reset();

    _subscriber_uri.clear();
    _dealer_uri.clear();
}

EcZmqPdo::EcZmqPdo( int32_t id, uint32_t type, const std::string zmq_uri):
_id(id),
_type(type),
_zmq_uri(zmq_uri)
{   
    _esc_name = iit::ecat::esc_type_map.at(type);
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

void EcZmqPdo::init(void)
{
    
}

int EcZmqPdo::write_connect(void)
{
    return 0;
}

int EcZmqPdo::write_quit(void)
{
    return 0;
}

int EcZmqPdo::read()
{
    try{
        int32_t size = zmq_client().extract_rx_pdo(pb_rx_pdos);
        if(size>0){
            get_from_pb();
        }
        return size;
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
