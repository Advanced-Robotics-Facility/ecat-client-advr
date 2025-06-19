#ifndef EC_ZIPC_H
#define EC_ZIPC_H

#include "utils/ec_thread.h"
#include "mechanism/zmq/ec_zmq_cmd.h"
#include "mechanism/common/ec_pdo.h"

class EcZipc : public EcZmqCmd,EcPdo<EcZmqPdo>,EcThread
{
public:

    EcZipc(std::string host_address,uint32_t host_port);
    ~EcZipc();

    void start_client(uint32_t period_ms) final;
    void stop_client() final ;
    void set_loop_time(uint32_t period_ms) final;
    void write() final;
    
public:
    
    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
    
private:
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
};

#endif
