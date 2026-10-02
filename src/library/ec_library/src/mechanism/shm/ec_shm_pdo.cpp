#include <esc_info.h>
#include "mechanism/shm/ec_shm_pdo.h"

EcShmPdo::EcShmPdo( int32_t id, uint32_t type, const std::string robot_name) :
    id(id), type(type)
{
    auto esc_name = iit::ecat::esc_type_map.at(type);
}

EcShmPdo::EcShmPdo( int32_t id, const std::string esc_name, const std::string robot_name) :
    id(id)
{
    name = std::string("iface_id_") + std::to_string(id);
}

EcShmPdo::EcShmPdo( int32_t id, uint32_t type, std::string rd_pp_name, std::string wr_pp_name) :
    id(id), type(type), rd_pp_name(rd_pp_name), wr_pp_name(wr_pp_name)
{
    name = std::string("iface_id_") + std::to_string(id);
}

void EcShmPdo::init(void){
    dev_info.id = id;
}

int EcShmPdo::read(void)  {
    int32_t nbytes = client_transport().extract_rx_pdo(pb_rx_pdos);
    if(client_transport().get_proto() == PROTOBUFF ){
        if(nbytes>0){
            get_from_pb();
        }
    }
    return nbytes;
}

int EcShmPdo::write(void) {
    if(client_transport().get_proto() == PROTOBUFF){
        set_to_pb();
    }

    return client_transport().push_tx_queue(dev_info,pb_tx_pdos);
}

int EcShmPdo::write_dummy(void) {
    return 0;
}

int EcShmPdo::write_connect(void) {
    return 0;
}

int EcShmPdo::write_quit(void) {
    return 0;
}
