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
    
    int read(void);
    int write(void);
    
    /**
    * @brief Return ZMQ URI for the ZMQ communication. 
    * 
    * @return std::__cxx11::string
    */
    std::string get_zmq_pdo_uri();

protected:    
    iit::advr::Ec_slave_pdo pb_rx_pdos,  pb_tx_pdos;    

//     virtual void get_from_pb(void) = 0;
//     virtual void set_to_pb(void) = 0;

private:
    
    zmq::multipart_t _multipart;
    
    std::string _zmq_uri;
    
    std::shared_ptr<zmq::context_t> _context;
    std::shared_ptr<zmq::socket_t>  _subscriber;
};

#endif

