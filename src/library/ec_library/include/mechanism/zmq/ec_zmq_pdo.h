#ifndef __EC_ZMQ_PDO__
#define __EC_ZMQ_PDO__

#include <charconv>
#include "mechanism/zmq/ec_zmq_context.h"

inline constexpr std::size_t PROTO_HEADER_BYTES = sizeof(std::uint32_t);
static constexpr size_t PB_MAX_BYTES = 512;

class EcZmqClient {
    public:
        static EcZmqClient& instance()
        {
            static EcZmqClient value;
            return value;
        }
    
        EcZmqClient(const EcZmqClient&) = delete;
        EcZmqClient& operator=(const EcZmqClient&) = delete;
    
        EcZmqClient(EcZmqClient&&) = delete;
        EcZmqClient& operator=(EcZmqClient&&) = delete;
    
        void initialize(std::string subscriber_uri,std::string dealer_uri);
        void stop();

        template<typename Dispatch>
        void recv(Dispatch&& dispatch_to_slave)
        {
            if (!_subscriber) {
                throw std::runtime_error("subscriber not initialized");
            }
        
            while (true) {
                const auto sub_id_result = _subscriber->recv(_zmq_sub_slot.zmq_sub_id, zmq::recv_flags::dontwait);

                if (!sub_id_result) {
                    break; // Socket queue completely drained
                }
        
                if (!_zmq_sub_slot.zmq_sub_id.more()) {
                    continue;
                }
        
                const auto sub_msg_result = _subscriber->recv(_zmq_sub_slot.zmq_sub_msg,zmq::recv_flags::none);
        
                if (!sub_msg_result) {
                    break;
                }
        
                const auto* first = static_cast<const char*>(_zmq_sub_slot.zmq_sub_id.data());
                const auto* last  = first + _zmq_sub_slot.zmq_sub_id.size();
        
                const auto result = std::from_chars(first, last, _zmq_sub_slot.slave_id);
        
                if (result.ec != std::errc{} || result.ptr != last) {
                    _zmq_sub_slot.slave_id = 0;
                }
        
                if(_zmq_sub_slot.slave_id == 0){
                    continue;
                }
        
                dispatch_to_slave(_zmq_sub_slot.slave_id);
            }
        }

        template<typename ProtoPdo>
        int extract_rx_pdo(ProtoPdo& pb_rx_pdo)
        {
            const std::size_t frame_size = _zmq_sub_slot.zmq_sub_msg.size();

            if (frame_size <= PROTO_HEADER_BYTES) {
                return -1;
            }

            const std::size_t payload_size = frame_size - PROTO_HEADER_BYTES;

            const auto* frame = static_cast<const std::uint8_t*>(_zmq_sub_slot.zmq_sub_msg.data());

            pb_rx_pdo.Clear();

            if (!pb_rx_pdo.ParseFromArray(frame + PROTO_HEADER_BYTES,static_cast<int>(payload_size))) {
                return -1;
            }

            return static_cast<int>(payload_size);
        }

        template<typename ProtoPdo>
        bool send(std::uint32_t slave_id,const ProtoPdo& pb_tx_pdo)
        {
            if (!_dealer) {
                return false;
            }

            const std::size_t payload_size = pb_tx_pdo.ByteSizeLong();
            const std::size_t frame_size = PROTO_HEADER_BYTES + payload_size;

            if (payload_size == 0 || frame_size > PB_MAX_BYTES) {
                return false;
            }

            zmq::message_t payload_frame(frame_size);

            auto* buffer = static_cast<std::uint8_t*>(payload_frame.data());

            const auto payload_u32 = static_cast<std::uint32_t>(payload_size);

            // Four-byte little-endian payload length.
            buffer[0] = static_cast<std::uint8_t>(payload_u32 & 0xFFu);
            buffer[1] = static_cast<std::uint8_t>((payload_u32 >> 8u) & 0xFFu);
            buffer[2] = static_cast<std::uint8_t>((payload_u32 >> 16u) & 0xFFu);
            buffer[3] = static_cast<std::uint8_t>((payload_u32 >> 24u) & 0xFFu);

            if (!pb_tx_pdo.SerializeToArray(buffer + PROTO_HEADER_BYTES,static_cast<int>(payload_size))) {
                return false;
            }

            const std::string topic = std::to_string(slave_id);

            if (!_dealer->send(zmq::buffer(topic),zmq::send_flags::sndmore)) {
                return false;
            }
            
            return _dealer->send(payload_frame,zmq::send_flags::none).has_value();
        }

    private:
        EcZmqClient();

        struct ZmqSubSlot{
            uint32_t slave_id;
            zmq::message_t zmq_sub_id;
            zmq::message_t zmq_sub_msg;
        };
        ZmqSubSlot _zmq_sub_slot = {};
        
        std::string _subscriber_uri = "";
        std::unique_ptr<zmq::socket_t> _subscriber;
        void init_subscriber(const std::string& subscriber_uri);
    
        zmq::message_t _zmq_deal_id,_zmq_deal_msg;
        std::string _dealer_uri = "";
        std::unique_ptr<zmq::socket_t> _dealer;
        void init_dealer( const std::string& dealer_uri);
};

inline EcZmqClient& zmq_client()
{
    return EcZmqClient::instance();
}

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

    void set_dev_type(uint8_t dev_type) {}
    
    int read(void);
    int write(void);
    void init(void);
    int write_connect(void);
    int write_quit(void);

protected:    
    iit::advrf::Ec_slave_pdo pb_rx_pdos,  pb_tx_pdos;  
    std::string name;

    virtual void get_from_pb(void) = 0;
    virtual void set_to_pb(void) = 0;

private:
    int32_t _id; 
    uint32_t _type;
    std::string _esc_name,_zmq_uri;
    std::string _msg_id;
    std::string _id_str;
    zmq::message_t _zmq_msg_id,_zmq_msg;
};

#endif

