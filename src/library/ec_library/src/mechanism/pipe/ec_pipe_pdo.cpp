#include <esc_info.h>
#include "mechanism/pipe/ec_pipe_pdo.h"

EcPipePdo::EcPipePdo( int32_t id, uint32_t type, const std::string robot_name) :
    id(id), type(type)
{
    auto esc_name = iit::ecat::esc_type_map.at(type);
    rd_pp_name = make_pipe_name(robot_name,esc_name,id,"tx");
    wr_pp_name = make_pipe_name(robot_name,esc_name,id,"rx");
    name = std::string("iface_id_") + std::to_string(id);
}

EcPipePdo::EcPipePdo( int32_t id, const std::string esc_name, const std::string robot_name) :
    id(id), type(type)
{
    rd_pp_name = make_pipe_name(robot_name,esc_name,id,"tx");
    wr_pp_name = make_pipe_name(robot_name,esc_name,id,"rx");
    name = std::string("iface_id_") + std::to_string(id);
}

EcPipePdo::EcPipePdo( int32_t id, uint32_t type, std::string rd_pp_name, std::string wr_pp_name) :
    id(id), type(type), rd_pp_name(rd_pp_name), wr_pp_name(wr_pp_name)
{
    name = std::string("iface_id_") + std::to_string(id);
}

void EcPipePdo::init(void)
{
    int ret_bind, ret_conn;
    // bind on rd iddp ==> read FROM
    if ( rd_iddp.bind(rd_pp_name, 0) <= 0 ) {
        throw(EcPipePdoError(EcPipePdoErrorNum::BIND_FAIL, rd_pp_name));
    }
    // connect to wr iddp ==> write TO
    if ( wr_iddp.connect(wr_pp_name) <= 0 ){
        throw(EcPipePdoError(EcPipePdoErrorNum::CONNECT_FAIL, wr_pp_name));
    }
}

int EcPipePdo::read(void)  {
    int ret = read_pb_from(rd_iddp, pb_buf, sizeof(pb_buf), &pb_rx_pdos, name );
    if ( ret > 0 ) {
        get_from_pb();
    }
    return ret;
}
int EcPipePdo::write(void) {
    pb_tx_pdos.Clear();
    set_to_pb();
    return write_pb_to (wr_iddp, pb_buf, sizeof(pb_buf), &pb_tx_pdos, name );
}

int EcPipePdo::write_dummy(void) {
    pb_tx_pdos.Clear();
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::DUMMY);
    return write_pb_to (wr_iddp, pb_buf, sizeof(pb_buf), &pb_tx_pdos, name );
}

int EcPipePdo::write_connect(void) {
    pb_tx_pdos.Clear();
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::CLIENT_PIPE);
    pb_tx_pdos.mutable_client_pdo()->set_type(iit::advr::Client_pipe::CONNECT);
    return write_pb_to (wr_iddp, pb_buf, sizeof(pb_buf), &pb_tx_pdos, name );
}

int EcPipePdo::write_quit(void) {
    pb_tx_pdos.Clear();
    set_pbHeader(pb_tx_pdos.mutable_header(), name, 0);
    pb_tx_pdos.set_type(iit::advr::Ec_slave_pdo::CLIENT_PIPE);
    pb_tx_pdos.mutable_client_pdo()->set_type(iit::advr::Client_pipe::QUIT);
    return write_pb_to (wr_iddp, pb_buf, sizeof(pb_buf), &pb_tx_pdos, name );
}
