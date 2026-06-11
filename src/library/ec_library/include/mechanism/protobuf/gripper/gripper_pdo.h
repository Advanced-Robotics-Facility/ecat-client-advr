#ifndef __GRIPPER_PDO__
#define __GRIPPER_PDO__

#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"

namespace GripperPdoRx {
    static const std::vector<std::string> name = {
        "status_word",
        "motor_pos",  
        "link_pos",   
        "pos_ref_fb", 
        "vel_ref_fb",  
        "tor_ref_fb",
        "fault"
    };
    static const int pdo_size = 7;
    using pdo_t = std::tuple<uint32_t, float, float, float, float, float, uint32_t>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t &pdo_tuple, std::vector<T> &pdo_vector) {
        if (pdo_vector.size() != pdo_size) return false;
        pdo_vector[0] = static_cast<T>(std::get<0>(pdo_tuple)); // statusword
        pdo_vector[1] = static_cast<T>(std::get<1>(pdo_tuple)); // motor_pos
        pdo_vector[2] = static_cast<T>(std::get<2>(pdo_tuple)); // motor_link
        pdo_vector[3] = static_cast<T>(std::get<3>(pdo_tuple)); // demanded_pos
        pdo_vector[4] = static_cast<T>(std::get<4>(pdo_tuple)); // demanded_vel
        pdo_vector[5] = static_cast<T>(std::get<5>(pdo_tuple)); // demanded_torque
        pdo_vector[6] = static_cast<T>(std::get<6>(pdo_tuple)); // error_code
        return true;
    }
};

namespace GripperPdoTx {
    static const std::vector<std::string> name = {
        "pos_ref",   
        "vel_ref",
        "tor_ref",
        "gain_0",
        "gain_1",
        "gain_2",
        "gain_3",
        "gain_4"
    };
    static const int pdo_size = 8;
    using pdo_t = std::tuple<float, float, float, float, float, float, float, float>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t &pdo_tuple, std::vector<T> &pdo_vector) {
        if (pdo_vector.size() != pdo_size) return false;
        pdo_vector[0] = static_cast<T>(std::get<0>(pdo_tuple)); 
        pdo_vector[1] = static_cast<T>(std::get<1>(pdo_tuple)); 
        pdo_vector[2] = static_cast<T>(std::get<2>(pdo_tuple)); 
        pdo_vector[3] = static_cast<T>(std::get<3>(pdo_tuple)); 
        pdo_vector[4] = static_cast<T>(std::get<4>(pdo_tuple)); 
        pdo_vector[5] = static_cast<T>(std::get<5>(pdo_tuple)); 
        pdo_vector[6] = static_cast<T>(std::get<6>(pdo_tuple)); 
        pdo_vector[7] = static_cast<T>(std::get<7>(pdo_tuple)); 
        return true;
    }
};

template <class T>
class GripperPdo : public T {

public:
    GripperPdo(const std::string, int32_t id, uint32_t type);
    ~GripperPdo();

    void get_from_pb();
    void set_to_pb();

    GripperPdoRx::pdo_t rx_pdo = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};
    GripperPdoTx::pdo_t tx_pdo = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    bool init_rx_pdo = false;
private:
    void init_pb();
};

template < class T >
inline void GripperPdo<T>::init_pb() 
{
   uint8_t  pb_buf[MAX_PB_SIZE];
   uint32_t msg_size=0;

   set_to_pb();
   msg_size = T::pb_tx_pdos.ByteSizeLong();
   T::pb_tx_pdos.SerializeToArray( (void*)(pb_buf+sizeof(msg_size)), msg_size);
}

template <class T>
inline GripperPdo<T>::GripperPdo(const std::string value, int32_t id, uint32_t type) :
    T(id, type, value)
{
    init_pb();
    T::init();
    T::write_connect();
}

template <class T>
inline GripperPdo<T>::~GripperPdo()
{
    T::write_quit();
}

template <class T>
inline void GripperPdo<T>::get_from_pb()
{
    std::get<0>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->statusword();
    std::get<1>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->motor_pos();    
    std::get<2>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->link_pos();    
    std::get<3>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->demanded_pos(); 
    std::get<4>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->demanded_vel();
    std::get<5>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->demanded_torque(); 
    std::get<6>(rx_pdo) = T::pb_rx_pdos.mutable_gripper_rx_pdo()->error_code();  

    if (!init_rx_pdo) {
        init_rx_pdo = true;
    }
}

template <class T>
inline void GripperPdo<T>::set_to_pb()
{
    set_pbHeader(T::pb_tx_pdos.mutable_header(), T::name, 0);

    T::pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::TX_GRIPPER_PDO);
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_target_pos(std::get<0>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_target_vel(std::get<1>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_target_torque(std::get<2>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_gain_0(std::get<3>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_gain_1(std::get<4>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_gain_2(std::get<5>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_gain_3(std::get<6>(tx_pdo));
    T::pb_tx_pdos.mutable_gripper_tx_pdo()->set_gain_4(std::get<7>(tx_pdo));
}

template class GripperPdo<EcPipePdo>;
template class GripperPdo<EcZmqPdo>;

#endif
