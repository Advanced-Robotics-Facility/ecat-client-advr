#ifndef __EC_ZMQ_PDO__
#define __EC_ZMQ_PDO__

#include <protobuf/repl_cmd.pb.h>
#include <protobuf/ecat_pdo.pb.h>

#include "zmq.hpp"
#include <zmq_addon.hpp>



class EcZmqPdo
{
public:
    
    typedef std::shared_ptr<EcZmqPdo> Ptr;
    
    /**
    * @brief Constructor of EC_Client_PDO Class.
    * 
    * @param zmq_uri p_zmq_uri: ZMQ URI (TCP/UDP) for zmq communication for the PDO reading.
    */
    EcZmqPdo(std::string zmq_uri);
    
    
    /**
    * @brief Destructor of EC_Client_PDO Class.
    * 
    */
    ~EcZmqPdo(){
        
    };
    
    /**
    * @brief Method to receive the PDOs using the ZMQ communication.
    * 
    * @param msg p_msg: Header of the ZMQ message (Name of the Slave).
    * @param ecat_pdo p_ecat_pdo: General slave pdo message serialized
    * Using the type of the slave, it's possible to de-serialized the message in a proper way.
    * @return bool
    */
    bool zmq_recv_pdo(std::string& msg,iit::advr::Ec_slave_pdo& ecat_pdo);
    
    /**
    * @brief Return ZMQ URI for the ZMQ communication. 
    * 
    * @return std::__cxx11::string
    */
    std::string get_zmq_pdo_uri();
    
private:
    
    zmq::multipart_t _multipart;
    
    std::string _zmq_uri;
    
    std::shared_ptr<zmq::context_t> _context;
    std::shared_ptr<zmq::socket_t>  _subscriber;
};

#endif

