#ifndef __MOTOR_PDO__
#define __MOTOR_PDO__

#include<tuple>
#include <pb_utils.h>
#include "mechanism/pipe/ec_pipe_pdo.h"
#include "mechanism/zmq/ec_zmq_pdo.h"
namespace MotorPdoRx{
    static const std::vector<std::string>name = {"link_pos", "motor_pos", "link_vel",
                                                 "motor_vel", "torque", "motor_temp",
                                                 "board_temp","fault","rtt","op_idx_ack","aux","cmd_aux_sts"};
    static const int pdo_size=12;
    using pdo_t= std::tuple<float, float, float, float,float,float,float,uint32_t, uint32_t,uint32_t, float, uint32_t>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t pdo_tuple,std::vector<T>& pdo_vector){
        if(pdo_vector.size()!=pdo_size){
           return false;
        }
        pdo_vector[0]= std::get<0>(pdo_tuple);
        pdo_vector[1]= std::get<1>(pdo_tuple);
        pdo_vector[2]= std::get<2>(pdo_tuple);
        pdo_vector[3]= std::get<3>(pdo_tuple);
        pdo_vector[4]= std::get<4>(pdo_tuple);
        pdo_vector[5]= std::get<5>(pdo_tuple);
        pdo_vector[6]= std::get<6>(pdo_tuple);
        pdo_vector[7]= std::get<7>(pdo_tuple);
        pdo_vector[8]= std::get<8>(pdo_tuple);
        pdo_vector[9]= std::get<7>(pdo_tuple);
        pdo_vector[10]= std::get<10>(pdo_tuple);
        pdo_vector[11]= std::get<11>(pdo_tuple);
        return true;
    }
};

namespace MotorPdoTx{
    static const std::vector<std::string>name = {"ctrl_type","pos_ref","vel_ref","tor_ref",
                                                 "gains_0", "gains_1","gains_2","gains_3","gains_4",
                                                 "op","idx","aux"};
    static const int pdo_size=12;
    using pdo_t= std::tuple<int32_t,float, float, float, float,float,float,float,float,uint32_t, uint32_t, float>;
    template <typename T>
    inline bool make_vector_from_tuple(const pdo_t pdo_tuple,std::vector<T>& pdo_vector){
        if(pdo_vector.size()!=pdo_size){
           return false;
        }
        pdo_vector[0]= std::get<0>(pdo_tuple);
        pdo_vector[1]= std::get<1>(pdo_tuple);
        pdo_vector[2]= std::get<2>(pdo_tuple);
        pdo_vector[3]= std::get<3>(pdo_tuple);
        pdo_vector[4]= std::get<4>(pdo_tuple);
        pdo_vector[5]= std::get<5>(pdo_tuple);
        pdo_vector[6]= std::get<6>(pdo_tuple);
        pdo_vector[7]= std::get<7>(pdo_tuple);
        pdo_vector[8]= std::get<8>(pdo_tuple);
        pdo_vector[9]= std::get<7>(pdo_tuple);
        pdo_vector[10]= std::get<10>(pdo_tuple);
        pdo_vector[11]= std::get<11>(pdo_tuple);
        return true;
    }
};

template <class T>
class MotorPdo: public T{

public:

    MotorPdo(const std::string ,int32_t id);
    MotorPdo(const std::string ,int32_t id, uint32_t type);
    ~MotorPdo();

    virtual void get_from_pb(void)=0;

    virtual void set_to_pb(void)=0;
    
    MotorPdoRx::pdo_t rx_pdo={0,0,0,0,0,0,0,0,0,0,0,0};
    float read_pos_ref,read_vel_ref,read_torque_ref,read_curr_ref; // should be added in rx_pdo
    bool init_rx_pdo=false;

    MotorPdoRx::pdo_t tx_pdo={0,0,0,0,0,0,0,0,0,0,0,0};
    float curr_ref; // should be added in tx_pdo
};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int id):
                           T(id,"Motor",value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline MotorPdo<T>::MotorPdo(std::string value,int32_t id, uint32_t type):
                           T(id, type, value)
{
    T::init();
    T::write_connect();
};

template < class T >
inline MotorPdo<T>::~MotorPdo()
{
   T::write_quit();
};


template class MotorPdo<EcPipePdo>;
template class MotorPdo<EcZmqPdo>;

#endif
