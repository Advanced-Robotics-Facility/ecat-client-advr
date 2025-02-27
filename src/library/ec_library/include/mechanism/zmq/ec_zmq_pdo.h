#ifndef __EC_ZMQ_PDO__
#define __EC_ZMQ_PDO__

#include "mechanism/zmq/ec_zmq_context.h"

class EcZmqPdo
{
public:
    
    typedef std::shared_ptr<EcZmqPdo> Ptr;
    
    /**
    * @brief Constructor of EC_Client_PDO Class.
    * 
    * @param zmq_uri p_zmq_uri: ZMQ URI (TCP/UDP) for zmq communication for the PDO reading.
    */
    
    EcZmqPdo( int32_t id, uint32_t type, const std::string);
    
    EcZmqPdo(int32_t id, const std::string esc_name, const std::string);

    EcZmqPdo(const std::string);
    
    /**
    * @brief Destructor of EC_Client_PDO Class.
    * 
    */
    virtual ~EcZmqPdo(){
        
    };
    
    int read(void);
    int write(void);
    void init(void);
    int write_connect(void);
    int write_quit(void);
    
    /**
    * @brief Return ZMQ URI for the ZMQ communication. 
    * 
    * @return std::__cxx11::string
    */
    std::string get_zmq_pdo_uri();

protected:    
    iit::advr::Ec_slave_pdo pb_rx_pdos,  pb_tx_pdos;  
    std::string name;

    virtual void get_from_pb(void) = 0;
    virtual void set_to_pb(void) = 0;

private:
    int32_t _id; 
    uint32_t _type;
    std::string _esc_name,_zmq_uri;
    std::string _msg_id;
    zmq::message_t _zmq_msg_id,_zmq_msg;
    std::shared_ptr<zmq::socket_t>  _subscriber;
};

#endif

