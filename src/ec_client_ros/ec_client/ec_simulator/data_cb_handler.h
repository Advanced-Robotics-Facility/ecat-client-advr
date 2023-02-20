#ifndef _DATA_CB_HANDLER_
#define _DATA_CB_HANDLER_

#include <pb_utils.h>
#include <esc_pipe_iface.h>

#include <esc_info.h>
#include <esc/hyq_iov5_esc.h>
#include <esc/mc_centAC_esc.h>
#include <esc/ft6_msp432_esc.h>
#include <esc/power_f28m36_board.h>
#include <esc/test_esc.h>

#include <protobuf/ecat_pdo.pb.h>

using DataHandlerType = void (*)(const uint32_t, const SH_PIFACE &,bool);
using DataHandlerTableType = std::map<uint32_t, DataHandlerType>;


void Default_pb_handle(const uint32_t, const SH_PIFACE &, bool read_only);


/**
 * @brief 
 * 
 * @tparam PDO_TYPE 
 * @param id 
 * @param tupla 
 */
template<typename PDO_TYPE>
void Default_raw_handle(const uint32_t id, const SH_PIFACE &esc_iface,bool read_only) {
    
    typedef typename PDO_TYPE::pdo_rx pdo_rx_t;
    typedef typename PDO_TYPE::pdo_tx pdo_tx_t;

    SH_IDP      rd_iddp, wr_iddp;
    uint32_t    esc_type = esc_iface->get_type();
    pdo_rx_t    pdo_rx;
    pdo_tx_t    pdo_tx;
    std::stringstream ss;
    
    rd_iddp = esc_iface->get_rd_iddp();
    wr_iddp = esc_iface->get_wr_iddp();
    
    // read
    uint32_t read_cnt = 0;
    int nbytes;
    do {
        // read "raw" binary data
        nbytes = rd_iddp->ddp_read(pdo_rx);
        if ( nbytes > 0) {
            read_cnt ++;
            ss << "[" << read_cnt << "]" << std::endl;
            ss << "read " << nbytes << " from " << rd_iddp->get_pipe_path() << std::endl;   
            ss << "rx_pdo " << pdo_rx << std::endl;
        }
    } while ( nbytes > 0);
    
    if(!read_only)
    {
        uint64_t tNow = iit::ecat::get_time_ns();
        pdo_tx.ts = ( uint16_t ) ( tNow /1000 );
        nbytes = wr_iddp->ddp_write(pdo_tx);
        ss << "write " << nbytes << " to " << wr_iddp->get_pipe_path() << std::endl;   
        ss << "tx_pdo " << pdo_tx << std::endl;
    }
    
    printf ( "%s", ss.str().c_str() );

}


#endif
