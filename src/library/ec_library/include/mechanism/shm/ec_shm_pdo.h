#ifndef __EC_PIPE_PDO__
#define __EC_PIPE_PDO__

#include <memory>

#include <pb_utils.h>
#include <shm_utils.hpp>

////////////////////////////////////////////////////
//
////////////////////////////////////////////////////
class EcShmPdo {

protected:

    bool                    initialized;
    std::string             name;
    int32_t                 id;
    uint32_t                type;    
    std::string             rd_pp_name, wr_pp_name;

    iit::advrf::Ec_slave_pdo pb_rx_pdos,  pb_tx_pdos;    
    uint8_t                 pb_buf_rd[MAX_PB_SIZE];
    uint8_t                 pb_buf_wr[MAX_PB_SIZE];
    DeviceInfo              dev_info = {};

    virtual void get_from_pb(void) = 0;
    virtual void set_to_pb(void) = 0;

public:

    EcShmPdo( int32_t id, uint32_t type, const std::string);
    
    EcShmPdo( int32_t id, const std::string esc_name, const std::string);

    EcShmPdo( int32_t id, uint32_t type,
                    std::string rd_pp_name,
                    std::string wr_pp_name );

    virtual ~EcShmPdo () {
    }
    
    virtual void init(void);

    iit::advrf::Ec_slave_pdo* pb_rx() { return &pb_rx_pdos; }
    iit::advrf::Ec_slave_pdo* pb_tx() { return &pb_tx_pdos; }

    std::string get_name() const    { return name; }
    uint32_t get_type() const       { return type; }

    void set_dev_type(uint8_t dev_type) {dev_info.type = static_cast<DeviceType>(dev_type);}

    int read(void);
    int write(void);
    int write_dummy(void);
    int write_connect(void);
    int write_quit(void);
};
#endif
